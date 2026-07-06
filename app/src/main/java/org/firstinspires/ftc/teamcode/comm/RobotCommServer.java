package org.firstinspires.ftc.teamcode.comm;

import com.google.gson.Gson;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Control Hub 안에서 도는 raw TCP + JSON 통신 서버 (offboard compute 링크).
 *
 * 구조 (분산 제어 원칙 — CLAUDE.md 규칙 6):
 *   - 이 서버의 모든 소켓 I/O는 백그라운드 스레드에서만 돈다.
 *   - 제어 루프(OpMode 메인 스레드)는 setPose()로 상태를 "넣기만", getLatestCommand()로
 *     명령을 "읽기만" 한다 — 절대 블로킹되지 않는다. WiFi 지연이 모터 루프를 늦추면 안 되므로.
 *
 * 프로토콜 (개행으로 구분된 JSON, newline-delimited JSON):
 *   Robot -> PC (50Hz 스트림):
 *     {"type":"pose","x":..,"y":..,"h":..,"slip":..,"robotTime":..}
 *   PC -> Robot:
 *     {"type":"ping","pcTime":<ms>}                 // 왕복지연 측정
 *     {"type":"goto","x":..,"y":..,"h":..}          // 목표 pose 명령
 *   Robot -> PC (ping 응답):
 *     {"type":"pong","pcTime":<원래값>,"robotTime":..}
 *
 * 좌표계/단위: x,y [m], h(heading) [rad], slip [rad/s] — Localizer/MecanumJacobian과 동일.
 *
 * 접속 방법: Control Hub가 AP일 때 IP는 보통 192.168.43.1, 포트는 DEFAULT_PORT.
 */
public class RobotCommServer {

    public static final int DEFAULT_PORT = 9999;

    /** PC가 보낸 목표 pose 명령 (immutable). */
    public static class Command {
        public final double x, y, h;   // m, m, rad
        public final long receivedAtRobotMillis;
        public Command(double x, double y, double h, long t) {
            this.x = x; this.y = y; this.h = h; this.receivedAtRobotMillis = t;
        }
    }

    private final int port;
    private final Gson gson = new Gson();

    private volatile boolean running = false;
    private ServerSocket serverSocket;
    private Thread acceptThread;

    // 최신 pose 스냅샷 (제어 루프가 쓰고, writer 스레드가 읽음)
    private volatile String latestPoseJson = null;
    // 최신 명령 (reader 스레드가 쓰고, 제어 루프가 읽음)
    private final AtomicReference<Command> latestCommand = new AtomicReference<>(null);
    // 현재 연결된 클라이언트 소켓 (writer/reader 공유)
    private volatile Socket clientSocket = null;

    public RobotCommServer() { this(DEFAULT_PORT); }
    public RobotCommServer(int port) { this.port = port; }

    /** 서버 시작 (백그라운드 accept 스레드 기동). OpMode init 단계에서 호출. */
    public void start() {
        if (running) return;
        running = true;
        acceptThread = new Thread(this::acceptLoop, "RobotCommServer-accept");
        acceptThread.setDaemon(true);
        acceptThread.start();
    }

    /** 서버 종료. OpMode stop()에서 반드시 호출해 소켓/스레드 정리. */
    public void stop() {
        running = false;
        closeQuietly(clientSocket);
        try { if (serverSocket != null) serverSocket.close(); } catch (IOException ignored) {}
    }

    /**
     * 제어 루프가 매 주기 호출 — 최신 pose를 JSON으로 만들어 저장(논블로킹).
     * 실제 전송은 writer 스레드가 알아서 함.
     */
    public void setPose(double x, double y, double h, double slip, long robotMillis) {
        Map<String, Object> m = new HashMap<>();
        m.put("type", "pose");
        m.put("x", x);
        m.put("y", y);
        m.put("h", h);
        m.put("slip", slip);
        m.put("robotTime", robotMillis);
        latestPoseJson = gson.toJson(m);
    }

    /** 제어 루프가 호출 — PC가 보낸 최신 목표 명령을 읽음(논블로킹). 없으면 null. */
    public Command getLatestCommand() {
        return latestCommand.get();
    }

    /** 클라이언트가 붙어 있는지 여부 (텔레메트리 표시용). */
    public boolean isClientConnected() {
        Socket s = clientSocket;
        return s != null && s.isConnected() && !s.isClosed();
    }

    // === 내부 스레드 로직 =====================================================

    private void acceptLoop() {
        try {
            serverSocket = new ServerSocket(port);
            serverSocket.setReuseAddress(true);
            while (running) {
                Socket sock;
                try {
                    sock = serverSocket.accept();   // 새 클라이언트 대기 (블로킹, 백그라운드라 OK)
                } catch (IOException e) {
                    if (!running) break;            // stop()에 의한 정상 종료
                    continue;
                }
                sock.setTcpNoDelay(true);           // 지연 최소화 (Nagle 끔)
                closeQuietly(clientSocket);         // 이전 연결 있으면 정리 (1클라이언트 정책)
                clientSocket = sock;

                // 이 클라이언트용 reader/writer 스레드 기동
                Thread reader = new Thread(() -> readLoop(sock), "RobotCommServer-reader");
                Thread writer = new Thread(() -> writeLoop(sock), "RobotCommServer-writer");
                reader.setDaemon(true); writer.setDaemon(true);
                reader.start(); writer.start();
            }
        } catch (IOException e) {
            // 포트 바인딩 실패 등 — 서버는 조용히 종료 (OpMode 텔레메트리에서 isClientConnected로 확인)
        }
    }

    /** PC -> Robot 수신 루프: ping/goto 처리. */
    private void readLoop(Socket sock) {
        try (BufferedReader in = new BufferedReader(
                new InputStreamReader(sock.getInputStream(), StandardCharsets.UTF_8))) {
            String line;
            while (running && (line = in.readLine()) != null) {
                handleMessage(sock, line);
            }
        } catch (IOException ignored) {
        } finally {
            closeQuietly(sock);
        }
    }

    /** Robot -> PC 송신 루프: 최신 pose를 ~50Hz로 스트리밍. */
    private void writeLoop(Socket sock) {
        String lastSent = null;
        try {
            OutputStream out = sock.getOutputStream();
            while (running && !sock.isClosed()) {
                String json = latestPoseJson;
                if (json != null && !json.equals(lastSent)) {
                    out.write((json + "\n").getBytes(StandardCharsets.UTF_8));
                    out.flush();
                    lastSent = json;
                }
                try { Thread.sleep(20); } catch (InterruptedException e) { break; } // ~50Hz
            }
        } catch (IOException ignored) {
        } finally {
            closeQuietly(sock);
        }
    }

    private void handleMessage(Socket sock, String line) {
        try {
            JsonObject o = new JsonParser().parse(line).getAsJsonObject();
            String type = o.has("type") ? o.get("type").getAsString() : "";
            switch (type) {
                case "ping": {
                    // 즉시 pong 회신 (왕복지연 측정용)
                    Map<String, Object> m = new HashMap<>();
                    m.put("type", "pong");
                    m.put("pcTime", o.get("pcTime").getAsLong());
                    m.put("robotTime", System.currentTimeMillis());
                    byte[] b = (gson.toJson(m) + "\n").getBytes(StandardCharsets.UTF_8);
                    synchronized (sock) { sock.getOutputStream().write(b); sock.getOutputStream().flush(); }
                    break;
                }
                case "goto": {
                    latestCommand.set(new Command(
                            o.get("x").getAsDouble(),
                            o.get("y").getAsDouble(),
                            o.get("h").getAsDouble(),
                            System.currentTimeMillis()));
                    break;
                }
                default:
                    // 알 수 없는 타입 무시
                    break;
            }
        } catch (Exception ignored) {
            // 깨진 JSON 한 줄은 무시하고 계속
        }
    }

    private static void closeQuietly(Socket s) {
        if (s != null) try { s.close(); } catch (IOException ignored) {}
    }
}
