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

        // 회전 제어 상태 (TurnTest와 동일한 래치/방향고정 로직)
        boolean headingLatched = false;
        double dirLock = 0.0;

        // 실기 검증: 위치+회전 동시제어 시 위치오차의 로컬프레임 변환이 회전중인 heading을
        // 계속 참조해 나선형 궤적으로 발산함. 2단계(회전 먼저→위치 이동)로 분리해 해결.
        // PHASE_TURN: 제자리 회전만. PHASE_MOVE: heading 고정하고 위치만 이동.
        String phase = "TURN";
        // 실기 검증: IMU 노이즈로 허용오차 안에 "순간적으로만" 스치는 순간이 있어 그걸로
        // 바로 MOVE 전환되면 아직 안 안정된 heading으로 이동 시작 → 궤적 발산.
        // 일정 시간(SETTLE_MS) 동안 계속 허용오차 안에 머물러야 진짜 전환.
        long settleStartMs = -1;
        final long SETTLE_MS = 300;

        try {
            while (opModeIsActive()) {
                loc.update();
                double x = loc.getX(), y = loc.getY(), th = loc.getHeading();

                // --- pose 스트리밍 (대시보드 표시용) ---
                server.setPose(x, y, th, 0.0, System.currentTimeMillis());

                // --- 대시보드가 보낸 최신 목표 확인 (새 목표 오면 회전 단계부터 재시작) ---
                RobotCommServer.Command cmd = server.getLatestCommand();
                if (cmd != null && (!hasTarget || cmd.x != tx || cmd.y != ty || cmd.h != tHeading)) {
                    tx = cmd.x; ty = cmd.y; tHeading = cmd.h;
                    hasTarget = true;
                    phase = "TURN";
                    headingLatched = false;
                    dirLock = 0.0;
                    settleStartMs = -1;
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

                    // --- 회전 제어: TurnTest(단독 실기검증 완료) 로직과 "동일"하게 맞춤 ---
                    // ±180 경계 노이즈로 좌우가 뒤집히는 걸 막기 위해, 오차가 큰 동안만(>DIR_LOCK_DEG)
                    // 회전 방향을 한 번 고정한다. "아직 미정(0)일 때 한 번만" 고정.
                    if (dirLock == 0.0 && Math.toDegrees(absHeadingErr) > DIR_LOCK_DEG) {
                        dirLock = Math.signum(eHeading);
                    }
                    // ★버그 수정 핵심: 허용오차 안에 "들어오는 즉시" dirLock 해제.
                    // (이전 버전은 settle 300ms 버틴 뒤에만 해제했음 → 그 사이 오버슈트하면
                    //  dirLock*absErr 강제부호가 실제오차 반대로 계속 밀어붙여 목표를 못 잡고
                    //  근처에서 헌팅/정체(‑176° 증상). TurnTest는 도달 즉시 풀어서 자연감쇠함.)
                    if (absHeadingErr < HEADING_TOLERANCE) {
                        dirLock = 0.0;
                    }

                    if (phase.equals("TURN")) {
                        // w 계산 — TurnTest와 완전히 동일: 방향 고정 시 부호 강제, 크기는 오차 비례.
                        double signedErr = (dirLock != 0.0) ? dirLock * absHeadingErr : eHeading;
                        w = clamp(SIGN * KP_HEADING * signedErr, -MAX_TURN, MAX_TURN);
                        // 목표 근처(≤RESUME_THRESHOLD)에선 최소출력 강제 안 함 → 자연 감쇠(오버슈트 방지).
                        // 그 밖에서만 정지마찰 데드존 보상용 최소출력 보장. (TurnTest와 동일 조건)
                        if (absHeadingErr <= RESUME_THRESHOLD) {
                            // 그대로 둠 (min power 미적용)
                        } else if (Math.abs(w) < MIN_TURN_POWER) {
                            w = Math.copySign(MIN_TURN_POWER, w);
                        }

                        // 전환 게이트: 허용오차 안에 SETTLE_MS 동안 "계속" 머물러야 MOVE로 전환.
                        // (순간적으로 스친 뒤 불안정한 heading으로 이동 시작하는 것을 막음)
                        if (absHeadingErr < HEADING_TOLERANCE) {
                            if (settleStartMs < 0) settleStartMs = System.currentTimeMillis();
                            if (System.currentTimeMillis() - settleStartMs >= SETTLE_MS) {
                                phase = "MOVE";
                                headingLatched = true;
                                dirLock = 0.0;
                                settleStartMs = -1;
                            }
                        } else {
                            settleStartMs = -1;
                        }
                    } else {
                        // MOVE 단계: 위치 이동, heading은 래치로 유지하고 크게 틀어질 때만 재보정.
                        if (headingLatched && absHeadingErr > RESUME_THRESHOLD) {
                            headingLatched = false;   // 많이 틀어짐 → heading 재보정 시작
                        } else if (!headingLatched && absHeadingErr < HEADING_TOLERANCE) {
                            headingLatched = true;    // 다시 정렬됨 → 재래치(위치-회전 커플링 축소)
                        }
                        boolean posReached = posErr < POS_TOLERANCE;
                        if (!posReached) {
                            vx = clamp(KP_POS * exLocal, -MAX_DRIVE, MAX_DRIVE);
                            vy = clamp(KP_POS * eyLocal, -MAX_DRIVE, MAX_DRIVE);
                        }
                        // heading이 이동 중 틀어지면 살짝만 보정 (래치 안 걸린 경우만).
                        // 실제 부호(eHeading)로 보정하므로 오버슈트해도 스스로 되돌아옴.
                        if (!headingLatched) {
                            w = clamp(SIGN * KP_HEADING * eHeading, -MAX_TURN * 0.5, MAX_TURN * 0.5);
                        }
                    }
                }

                drive.driveRobotRelative(vx, vy, w);

                telemetry.addData("client", server.isClientConnected() ? "연결됨" : "대기");
                telemetry.addData("phase", phase);
                telemetry.addData("pose", "X=%.3f Y=%.3f θ=%.1f°", x, y, Math.toDegrees(th));
                if (hasTarget) {
                    telemetry.addData("목표", "X=%.2f Y=%.2f θ=%.0f°", tx, ty, Math.toDegrees(tHeading));
                    telemetry.addData("남은거리", "%.3f m", posErr);
                    // 라이브 디버깅용: 회전 상태를 실시간으로 관찰 (스크린샷 사후분석보다 빠름)
                    telemetry.addData("heading오차(°)", "%.1f", Math.toDegrees(eHeading));
                    telemetry.addData("w 출력", "%.2f", w);
                    telemetry.addData("dirLock", "%.0f", dirLock);
                    long settleMs = (settleStartMs < 0) ? 0 : (System.currentTimeMillis() - settleStartMs);
                    telemetry.addData("settle(ms)", "%d / %d", settleMs, SETTLE_MS);
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
