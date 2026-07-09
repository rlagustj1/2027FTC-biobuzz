package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.io.File;
import java.io.FileWriter;

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

    // === 목표 좌표 (필드 기준, 시작점이 원점) — 연결 없이 위치+회전 통합 검증 ==========
    // 회전+이동 동시 목표로 DashboardDriveOpMode와 동일한 제어를 대시보드 없이 검증한다.
    private static final double TARGET_X = 0.3;               // m (전방)
    private static final double TARGET_Y = 0.3;              // m (좌측)
    // 회전 없이 위치만(홀로노믹 대각선 이동) — orbit/커플링 배제해 순수 위치제어 검증.
    // 회전이 필요하면 heading을 바꾸되, 이동-회전 커플링(orbit)은 별도 문제.
    private static final double TARGET_HEADING = Math.toRadians(0);

    // === P 게인 — 진단용으로 속도 낮춤(관성 오버슈트 배제) ======================
    private static final double SIGN = -1.0;                 // 회전 부호 (실기 확정)
    private static final double KP_POS = 0.9;                // 위치오차(m) → 속도
    private static final double KP_HEADING = 0.6;            // 방향오차(rad) → 회전
    private static final double MAX_DRIVE = 0.25;            // 병진 출력 상한 (느리게 → 슬립/관성↓)
    private static final double MAX_TURN = 0.35;             // 회전 출력 상한

    // === 도달/게이트 =====================================================
    private static final double POS_TOLERANCE = 0.03;        // m (3cm)
    private static final double HEADING_TOLERANCE = Math.toRadians(8);
    private static final double RESUME_THRESHOLD = Math.toRadians(14);
    private static final double MIN_TURN_POWER = 0.15;
    // 커플링 방지: heading 오차 크면(큰 회전 중) 이동 억제 → 회전 먼저, 그다음 이동+회전 동시
    private static final double MOVE_HEADING_GATE = Math.toRadians(25);

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

        boolean headingLatched = false;

        // 주행 중 pose를 로봇 내부 파일에 기록 (연결 불필요). 끝나면 adb pull /sdcard/gotolog.csv
        FileWriter fw = null;
        try {
            fw = new FileWriter(new File("/sdcard/gotolog.csv"));
            fw.write("t_ms,tx,ty,x,y,h_deg,vx,vy,w,posErr\n");
        } catch (Exception ignored) {}
        long t0 = System.currentTimeMillis();

        while (opModeIsActive()) {
            loc.update();

            double x = loc.getX();
            double y = loc.getY();
            double th = loc.getHeading();

            // --- 필드 프레임 위치 오차 → 로컬 프레임 ---
            double exField = TARGET_X - x;
            double eyField = TARGET_Y - y;
            double c = Math.cos(-th), s = Math.sin(-th);
            double exLocal = exField * c - eyField * s;
            double eyLocal = exField * s + eyField * c;
            double eHeading = normalizeAngle(TARGET_HEADING - th);
            double posErr = Math.hypot(exField, eyField);
            double absHeadingErr = Math.abs(eHeading);

            double vx = 0, vy = 0, w = 0;

            // --- 위치 제어: heading 얼추 맞을 때만(큰 회전 중엔 억제 → 커플링 방지) ---
            boolean headingAligned = absHeadingErr < MOVE_HEADING_GATE;
            if (posErr >= POS_TOLERANCE && headingAligned) {
                vx = clamp(KP_POS * exLocal, -MAX_DRIVE, MAX_DRIVE);
                vy = clamp(KP_POS * eyLocal, -MAX_DRIVE, MAX_DRIVE);
            }

            // --- heading 제어: 래치로 진동 방지 (DashboardDriveOpMode와 동일) ---
            if (headingLatched && absHeadingErr > RESUME_THRESHOLD) {
                headingLatched = false;
            } else if (!headingLatched && absHeadingErr < HEADING_TOLERANCE) {
                headingLatched = true;
            }
            if (!headingLatched) {
                w = clamp(SIGN * KP_HEADING * eHeading, -MAX_TURN, MAX_TURN);
                if (absHeadingErr > RESUME_THRESHOLD && Math.abs(w) < MIN_TURN_POWER) {
                    w = Math.copySign(MIN_TURN_POWER, w);
                }
            }

            boolean reached = posErr < POS_TOLERANCE && absHeadingErr < HEADING_TOLERANCE;
            drive.driveRobotRelative(vx, vy, w);

            // pose 로깅 (한 줄/루프)
            if (fw != null) {
                try {
                    fw.write(String.format("%d,%.3f,%.3f,%.4f,%.4f,%.1f,%.3f,%.3f,%.3f,%.4f\n",
                            System.currentTimeMillis() - t0, TARGET_X, TARGET_Y,
                            x, y, Math.toDegrees(th), vx, vy, w, posErr));
                } catch (Exception ignored) {}
            }

            telemetry.addData("상태", reached ? ">>> 목표 도달!" : "이동중...");
            telemetry.addData("목표", "X=%.2f Y=%.2f θ=%.0f°", TARGET_X, TARGET_Y, Math.toDegrees(TARGET_HEADING));
            telemetry.addData("pose", "X=%.3f Y=%.3f θ=%.1f°", x, y, Math.toDegrees(th));
            telemetry.addData("남은", "%.3f m, %.1f°", posErr, Math.toDegrees(eHeading));
            telemetry.addData("출력", "vx=%.2f vy=%.2f w=%.2f", vx, vy, w);
            telemetry.update();
        }

        drive.driveRobotRelative(0, 0, 0);
        if (fw != null) {
            try { fw.flush(); fw.close(); } catch (Exception ignored) {}
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
