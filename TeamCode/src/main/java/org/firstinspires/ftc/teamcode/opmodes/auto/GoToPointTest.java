package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * 경로추종 검증용: 목표 좌표 (x, y, heading)까지 P제어로 이동.
 * sim/odometry_ik_sim.py 의 path_follow_control 을 실물로 이식.
 *
 * 동작: START → 오도로 현재 pose 추적 → 목표까지 오차 P제어 → 도달하면 정지.
 *
 * ⚠️ 안전: 넓은 공간에서, 비상시 STOP 누를 준비하고 테스트할 것.
 *   출력이 크면 KP를 낮춰서 다시. 처음엔 짧은 목표(0.5m)로 시작 권장.
 */
@Autonomous(name = "GoTo Point Test", group = "auto")
public class GoToPointTest extends LinearOpMode {

    // === 목표 좌표 (필드 기준, 시작점이 원점) ==================================
    // TODO: 처음엔 짧게 (0.5m 전방) 테스트 후 늘릴 것
    private static final double TARGET_X = 0.5;               // m (전방)
    private static final double TARGET_Y = 0.0;               // m (좌측)
    private static final double TARGET_HEADING = 0.0;         // rad (+CCW)

    // === P 게인 (실기 튜닝 필요 — CLAUDE.md 규칙 1) ============================
    // TODO: 실측 튜닝 — 진동하면 낮추고, 너무 느리면 올림. 시뮬 초기값에서 시작.
    private static final double KP_POS = 1.2;                 // 위치오차(m) → 속도
    private static final double KP_HEADING = 1.5;             // 방향오차(rad) → 회전
    private static final double MAX_DRIVE = 0.5;              // 병진 출력 상한 (안전)
    private static final double MAX_TURN = 0.4;               // 회전 출력 상한

    // === 도달 판정 허용오차 =====================================================
    private static final double POS_TOLERANCE = 0.03;         // m (3cm)
    private static final double HEADING_TOLERANCE = Math.toRadians(3); // 3도

    @Override
    public void runOpMode() {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        // 데드휠 물리 배치가 반대라 순서 스왑 (OdometryTest와 동일)
        Localizer loc = new Localizer(hardwareMap, "perpEncoder", "parallelEncoder");

        drive.setOpenLoop();

        telemetry.addLine("GoTo Point. 목표: X=" + TARGET_X + " Y=" + TARGET_Y);
        telemetry.addLine("START 누르면 이동. 비상시 STOP!");
        telemetry.update();

        waitForStart();
        loc.resetPose();

        while (opModeIsActive()) {
            loc.update();

            double x = loc.getX();
            double y = loc.getY();
            double th = loc.getHeading();

            // --- 필드 프레임 위치 오차 ---
            double exField = TARGET_X - x;
            double eyField = TARGET_Y - y;

            // --- 필드 오차 → 로봇 로컬 프레임 (역회전) ---
            double c = Math.cos(-th), s = Math.sin(-th);
            double exLocal = exField * c - eyField * s;
            double eyLocal = exField * s + eyField * c;

            // --- 방향 오차 (최단 회전으로 정규화) ---
            double eHeading = normalizeAngle(TARGET_HEADING - th);

            // --- P 제어 + 출력 제한 ---
            double vx = clamp(KP_POS * exLocal, -MAX_DRIVE, MAX_DRIVE);
            double vy = clamp(KP_POS * eyLocal, -MAX_DRIVE, MAX_DRIVE);
            // 실기 검증: 회전 폭주(양의 피드백) → 드라이브 w 부호가 IMU θ와 반대여서
            // eHeading에 -1을 곱해 부호 정합 (음의 피드백으로 교정).
            double w = clamp(-KP_HEADING * eHeading, -MAX_TURN, MAX_TURN);

            // --- 도달 판정 ---
            double posErr = Math.hypot(exField, eyField);
            boolean reached = posErr < POS_TOLERANCE && Math.abs(eHeading) < HEADING_TOLERANCE;

            if (reached) {
                drive.driveRobotRelative(0, 0, 0);
                telemetry.addLine(">>> 목표 도달!");
            } else {
                drive.driveRobotRelative(vx, vy, w);
            }

            telemetry.addData("pose", "X=%.3f Y=%.3f θ=%.1f°", x, y, Math.toDegrees(th));
            telemetry.addData("목표까지", "%.3f m, %.1f°", posErr, Math.toDegrees(eHeading));
            telemetry.addData("출력", "vx=%.2f vy=%.2f w=%.2f", vx, vy, w);
            telemetry.update();
        }

        drive.driveRobotRelative(0, 0, 0);
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
