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

    // 위치 P 게인 (GoToPointTest 검증값)
    private static final double KP_POS = 1.2;
    private static final double MAX_DRIVE = 0.5;
    private static final double POS_TOLERANCE = 0.03;

    // 회전 P 게인 (TurnTest 실기 검증값 — SIGN, 방향고정, 근접구간 무강제 로직 포함)
    // SIGN 실기 확정: -1 (하드웨어상 +w=CW라 부호 반대). +1이면 heading이 목표에서 발산 → -176 정체.
    // TurnTest·GoToPointTest와 부호 통일함.
    private static final double SIGN = -1.0;
    private static final double KP_HEADING = 0.6;
    private static final double MAX_TURN = 0.45;
    private static final double MIN_TURN_POWER = 0.15;
    private static final double HEADING_TOLERANCE = Math.toRadians(8);
    private static final double RESUME_THRESHOLD = Math.toRadians(14);
    // 커플링 방지 게이트: heading 오차가 이 값보다 크면(큰 회전 중) 위치이동을 잠깐 끔.
    // 회전 먼저 얼추 맞춘 뒤 이동+회전 동시 → 회전 중 arc 발산 방지.
    private static final double MOVE_HEADING_GATE = Math.toRadians(25);
    private static final double DIR_LOCK_DEG = 160.0;

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

        // heading 래치(이력현상): 목표 근처 도달 시 걸어 진동(버징) 방지. TurnTest와 동일 개념.
        boolean headingLatched = false;

        try {
            while (opModeIsActive()) {
                loc.update();
                double x = loc.getX(), y = loc.getY(), th = loc.getHeading();

                // --- pose 스트리밍 (대시보드 표시용) ---
                server.setPose(x, y, th, 0.0, System.currentTimeMillis());

                // --- 대시보드가 보낸 최신 목표 확인 (새 목표 오면 래치 해제) ---
                RobotCommServer.Command cmd = server.getLatestCommand();
                if (cmd != null && (!hasTarget || cmd.x != tx || cmd.y != ty || cmd.h != tHeading)) {
                    tx = cmd.x; ty = cmd.y; tHeading = cmd.h;
                    hasTarget = true;
                    headingLatched = false;
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
                    double absHeadingErr = Math.abs(eHeading);

                    // ================= 단순 동시제어 (위치 X·Y·heading 한꺼번에) =================
                    // 부호(SIGN=-1)와 오도 오프셋을 실기로 잡았으므로, 2단계/dirLock/settle 같은
                    // 우회 로직을 걷어내고 GoToPointTest처럼 셋을 동시에 P제어한다. 메카넘은
                    // 홀로노믹이라 이동+회전이 동시에 가능하고, 매 루프 현재 heading으로 좌표변환하니
                    // 회전 중에도 위치명령이 알아서 갱신된다. (X만 되고 Y/heading 깨지던 원인 = 우회로직)

                    // --- 위치 제어: 로컬프레임 오차 → vx, vy ---
                    // 커플링 방지: heading이 많이 틀어져 있으면(큰 회전 중) 이동을 잠깐 끈다.
                    // 회전 중 오도 드리프트를 위치제어가 쫓아가며 곡선(arc) 그리던 것을 막음.
                    // heading이 게이트(±MOVE_GATE) 안으로 들어오면 그때부터 이동+회전 동시.
                    boolean headingAligned = absHeadingErr < MOVE_HEADING_GATE;
                    if (posErr >= POS_TOLERANCE && headingAligned) {
                        vx = clamp(KP_POS * exLocal, -MAX_DRIVE, MAX_DRIVE);
                        vy = clamp(KP_POS * eyLocal, -MAX_DRIVE, MAX_DRIVE);
                    }

                    // --- heading 제어: 래치로 목표 근처 진동(버징) 방지 (TurnTest와 동일 개념) ---
                    if (headingLatched && absHeadingErr > RESUME_THRESHOLD) {
                        headingLatched = false;                 // 많이 틀어짐 → 재보정 시작
                    } else if (!headingLatched && absHeadingErr < HEADING_TOLERANCE) {
                        headingLatched = true;                  // 도달 → 래치(진동 방지)
                    }
                    if (!headingLatched) {
                        w = clamp(SIGN * KP_HEADING * eHeading, -MAX_TURN, MAX_TURN);
                        // 목표 근처(≤RESUME)에선 최소출력 강제 안 함(오버슈트/버징 방지),
                        // 멀 때만 정지마찰 보상 최소출력. (TurnTest와 동일)
                        if (absHeadingErr > RESUME_THRESHOLD && Math.abs(w) < MIN_TURN_POWER) {
                            w = Math.copySign(MIN_TURN_POWER, w);
                        }
                    }
                }

                drive.driveRobotRelative(vx, vy, w);

                telemetry.addData("client", server.isClientConnected() ? "연결됨" : "대기");
                telemetry.addData("제어", "단순 동시제어 (X·Y·heading)");
                telemetry.addData("pose", "X=%.3f Y=%.3f θ=%.1f°", x, y, Math.toDegrees(th));
                if (hasTarget) {
                    telemetry.addData("목표", "X=%.2f Y=%.2f θ=%.0f°", tx, ty, Math.toDegrees(tHeading));
                    telemetry.addData("남은거리", "%.3f m", posErr);
                    // 라이브 디버깅용 실시간 값
                    telemetry.addData("heading오차(°)", "%.1f", Math.toDegrees(eHeading));
                    telemetry.addData("출력", "vx=%.2f vy=%.2f w=%.2f", vx, vy, w);
                    telemetry.addData("latch", headingLatched);
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
