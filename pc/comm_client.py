"""
외부 PC <-> Control Hub 통신 클라이언트 (offboard compute 링크).

Control Hub의 RobotCommServer(raw TCP + newline JSON)에 접속해서:
  - pose 스트림 수신 및 출력
  - 왕복지연(RTT) 측정 (ping/pong)
  - goto 목표 명령 전송

사용법 (Control Hub WiFi에 노트북 접속 후):
  python pc/comm_client.py                       # 기본 192.168.43.1:9999 접속
  python pc/comm_client.py --host 192.168.43.1 --port 9999
  대화형 입력:
    p            -> ping 보내서 RTT 측정
    g x y hdeg   -> goto 명령 (예: g 1.0 0.5 90)
    q            -> 종료

이후 여기에 경로최적화/비전 AI 로직을 얹으면, 수신한 pose로 계산해
goto 명령을 되돌려보내는 offboard 컨트롤러가 된다.
"""

import argparse
import json
import socket
import threading
import time

# 단위: x,y [m], h(heading) [rad], slip [rad/s] — 로봇 코드와 동일


class RobotLink:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None
        self.rx_file = None
        self.running = False
        self.latest_pose = None      # dict: x,y,h,slip,robotTime
        self.last_rtt_ms = None      # 가장 최근 측정된 왕복지연 (ms)
        self._ping_times = {}        # pcTime -> send monotonic time
        self._lock = threading.Lock()

    def connect(self):
        self.sock = socket.create_connection((self.host, self.port), timeout=5)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.rx_file = self.sock.makefile("r", encoding="utf-8")
        self.running = True
        threading.Thread(target=self._rx_loop, daemon=True).start()
        print(f"[연결됨] {self.host}:{self.port}")

    def _rx_loop(self):
        try:
            for line in self.rx_file:
                line = line.strip()
                if not line:
                    continue
                try:
                    msg = json.loads(line)
                except json.JSONDecodeError:
                    continue
                self._handle(msg)
        except OSError:
            pass
        finally:
            self.running = False
            print("[수신 종료]")

    def _handle(self, msg):
        mtype = msg.get("type")
        if mtype == "pose":
            with self._lock:
                self.latest_pose = msg
        elif mtype == "pong":
            pc_time = msg.get("pcTime")
            send_mono = self._ping_times.pop(pc_time, None)
            if send_mono is not None:
                rtt_ms = (time.perf_counter() - send_mono) * 1000.0
                self.last_rtt_ms = rtt_ms
                print(f"[RTT] 왕복지연 {rtt_ms:.1f} ms  (robotTime={msg.get('robotTime')})")

    def _send(self, obj):
        data = (json.dumps(obj) + "\n").encode("utf-8")
        self.sock.sendall(data)

    def ping(self):
        pc_time = int(time.time() * 1000)
        self._ping_times[pc_time] = time.perf_counter()
        self._send({"type": "ping", "pcTime": pc_time})

    def goto(self, x, y, h_rad):
        self._send({"type": "goto", "x": x, "y": y, "h": h_rad})
        print(f"[전송] goto x={x} y={y} h={h_rad:.3f}rad")

    def close(self):
        self.running = False
        # sock이 None(한 번도 연결 안 됨)이어도 안전하게 종료
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass


def pose_printer(link):
    """0.5초마다 최신 pose를 한 줄로 출력."""
    while link.running:
        with link._lock:
            p = link.latest_pose
        if p:
            print(f"  pose  x={p['x']:+.3f}  y={p['y']:+.3f}  "
                  f"h={p['h']:+.3f}rad  slip={p['slip']:+.3f}", end="\r")
        time.sleep(0.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.43.1")
    ap.add_argument("--port", type=int, default=9999)
    args = ap.parse_args()

    link = RobotLink(args.host, args.port)
    try:
        link.connect()
    except OSError as e:
        print(f"[접속 실패] {e}")
        print("Control Hub WiFi에 접속했는지, CommTestOpMode가 실행 중인지 확인하세요.")
        return

    threading.Thread(target=pose_printer, args=(link,), daemon=True).start()

    print("명령: p=ping(RTT측정)  g x y hdeg=goto  q=종료")
    try:
        while link.running:
            cmd = input().strip().split()
            if not cmd:
                continue
            if cmd[0] == "q":
                break
            elif cmd[0] == "p":
                link.ping()
            elif cmd[0] == "g" and len(cmd) == 4:
                x, y, hdeg = float(cmd[1]), float(cmd[2]), float(cmd[3])
                link.goto(x, y, hdeg * 3.141592653589793 / 180.0)
            else:
                print("사용법: p | g x y hdeg | q")
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        link.close()
        print("\n[종료]")


if __name__ == "__main__":
    main()
