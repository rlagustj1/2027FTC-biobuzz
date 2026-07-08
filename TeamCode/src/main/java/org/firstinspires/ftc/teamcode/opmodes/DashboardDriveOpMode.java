package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.comm.RobotCommServer;
import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * 대시보드 연동 주행 OpMode.
 * 외부 PC 대시보드에서 필드를 클릭하면(goto 명령) 로봇이 그 좌표로 이동한다.
 * 동시에 현재 pose를 통신서버로 스트리밍해 대시보드에 실시간 표시.
 *
 * 통신: RobotCommServer(TCP :9999). USB(adb forward)로 노트북과 연결.
 *   PC: adb forward tcp:9999 tcp:9999 → python pc/dashboard.py --host 127.0.0.1
 *
 * 구조 = GoToPointTest(P제어) + CommTestOpMode(통신) 통합.
 * 목표는 고정이 아니라 대시보드가 보낸 최신 goto로 갱신된다.
 *
 * ⚠️ 안전: 넓은 공간, STOP 손 근처. 대시보드로 먼 좌표 보내면 로봇이 그만큼 감.
 */
@TeleOp(name = "Dashboard Drive", group = "auto")
public class DashboardDriveOpMode extends LinearOpMode {

    // P 게인 (GoToPointTest와 동일 검증값)
    private static final double KP_POS = 1.2;
    private static final double KP_HEADING = 1.5;
    private static final double MAX_DRIVE = 0.5;
    private static final double MAX_TURN = 0.4;
    private static final double POS_TOLERANCE = 0.03;
    private static final double HEADING_TOLERANCE = Math.toRadians(3);

    @Override
    public void runOpMode() {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        Localizer loc = new Localizer(hardwareMap, "perpEncoder", "parallelEncoder");
        RobotCommServer server = new RobotCommServer();

        drive.setOpenLoop();
        server.start();

        telemetry.addLine("통신서버 시작 (:" + RobotCommServer.DEFAULT_PORT + ")");
        telemetry.addLine("PC: adb forward tcp:9999 tcp:9999 → python pc/dashboard.py --host 127.0.0.1");
        telemetry.addLine("START 후 대시보드 필드 클릭 → 로봇 이동");
        telemetry.update();

        waitForStart();
        loc.resetPose();

        // 목표 (null이면 정지 대기). 대시보드 goto 명령으로 갱신.
        boolean hasTarget = false;
        double tx = 0, ty = 0, tHeading = 0;

        try {
            while (opModeIsActive()) {
                loc.update();
                double x = loc.getX(), y = loc.getY(), th = loc.getHeading();

                // --- pose 스트리밍 (대시보드 표시용) ---
                server.setPose(x, y, th, 0.0, System.currentTimeMillis());

                // --- 대시보드가 보낸 최신 목표 확인 ---
                RobotCommServer.Command cmd = server.getLatestCommand();
                if (cmd != null) {
                    tx = cmd.x; ty = cmd.y; tHeading = cmd.h;
                    hasTarget = true;
                }

                double vx = 0, vy = 0, w = 0;
                double posErr = 0, eHeading = 0;

                if (hasTarget) {
                    double exField = tx - x, eyField = ty - y;
                    double c = Math.cos(-th), s = Math.sin(-th);
                    double exLocal = exField * c - eyField * s;
                    double eyLocal = exField * s + eyField * c;
                    eHeading = normalizeAngle(tHeading - th);
                    posErr = Math.hypot(exField, eyField);

                    boolean reached = posErr < POS_TOLERANCE && Math.abs(eHeading) < HEADING_TOLERANCE;
                    if (!reached) {
                        vx = clamp(KP_POS * exLocal, -MAX_DRIVE, MAX_DRIVE);
                        vy = clamp(KP_POS * eyLocal, -MAX_DRIVE, MAX_DRIVE);
                        w = clamp(-KP_HEADING * eHeading, -MAX_TURN, MAX_TURN); // 검증된 부호
                    }
                }

                drive.driveRobotRelative(vx, vy, w);

                telemetry.addData("client", server.isClientConnected() ? "연결됨" : "대기");
                telemetry.addData("pose", "X=%.3f Y=%.3f θ=%.1f°", x, y, Math.toDegrees(th));
                if (hasTarget) {
                    telemetry.addData("목표", "X=%.2f Y=%.2f θ=%.0f°", tx, ty, Math.toDegrees(tHeading));
                    telemetry.addData("남은거리", "%.3f m", posErr);
                } else {
                    telemetry.addLine("목표 대기중 (대시보드 클릭)");
                }
                telemetry.update();
            }
        } finally {
            drive.driveRobotRelative(0, 0, 0);
            server.stop();
        }
    }

    private static double normalizeAngle(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
