package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.comm.RobotCommServer;

/**
 * 통신 계층(RobotCommServer) 단독 검증용 OpMode.
 * 오도메트리 하드웨어가 아직 없어도 되도록, 실제 pose 대신 가짜 pose
 * (반지름 1m 원을 도는 로봇)를 생성해 외부 PC로 스트리밍한다.
 *
 * 학교 도착 후 첫 단계로 이걸 돌려서:
 *   1) 노트북을 Control Hub WiFi에 접속
 *   2) python pc/comm_client.py 실행 -> pose 수신 + 왕복지연(RTT) 확인
 *   3) 대시보드/명령 채널이 정상인지 goto 명령 왕복 확인
 * 이 통신 링크가 검증되면, 다음에 CommTestOpMode의 가짜 pose를 실제 Localizer로 교체.
 */
@TeleOp(name = "Comm Test (fake pose)", group = "Test")
public class CommTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotCommServer server = new RobotCommServer();
        server.start();

        telemetry.addLine("통신 서버 시작됨. 포트 " + RobotCommServer.DEFAULT_PORT);
        telemetry.addLine("외부 PC에서 192.168.43.1:" + RobotCommServer.DEFAULT_PORT + " 로 접속");
        telemetry.addLine("START를 누르면 가짜 pose 스트리밍 시작");
        telemetry.update();

        waitForStart();

        long t0 = System.currentTimeMillis();
        try {
            while (opModeIsActive()) {
                // 가짜 pose: 반지름 1m 원을 0.5rad/s로 도는 로봇 (통신 검증용 더미 데이터)
                double t = (System.currentTimeMillis() - t0) / 1000.0;
                double x = Math.cos(0.5 * t);
                double y = Math.sin(0.5 * t);
                double h = 0.5 * t;
                double slip = 0.0; // 가짜라 슬립 없음

                server.setPose(x, y, h, slip, System.currentTimeMillis());

                // PC가 보낸 목표 명령이 잘 들어오는지 텔레메트리로 확인
                RobotCommServer.Command cmd = server.getLatestCommand();

                telemetry.addData("client", server.isClientConnected() ? "연결됨" : "대기중");
                telemetry.addData("fake pose", "x=%.2f y=%.2f h=%.1f°", x, y, Math.toDegrees(h));
                if (cmd != null) {
                    telemetry.addData("last goto", "x=%.2f y=%.2f h=%.1f°",
                            cmd.x, cmd.y, Math.toDegrees(cmd.h));
                }
                telemetry.update();

                sleep(20); // ~50Hz
            }
        } finally {
            server.stop();
        }
    }
}
