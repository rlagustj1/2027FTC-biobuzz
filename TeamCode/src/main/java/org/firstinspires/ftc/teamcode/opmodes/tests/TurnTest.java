package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * 회전 제어 부호 검증용 OpMode.
 * 제자리에서 목표 heading(기본 +90°, CCW)으로만 회전한다. 병진(vx,vy)=0.
 *
 * 목적: heading P제어의 w 부호가 맞는지 확인.
 *   - 부호 맞음: 로봇이 목표각(+90°)으로 돌아가 멈춤 → 오차 0으로 수렴
 *   - 부호 틀림: 반대로 돌거나 계속 폭주 → 즉시 STOP
 *
 * SIGN 상수만 뒤집어 두 경우를 비교할 수 있게 함.
 *
 * ⚠️ 안전: 바퀴 땅에 닿게, 주변 여유공간, STOP 손 근처.
 */
@Autonomous(name = "Turn Test (heading)", group = "Test")
public class TurnTest extends LinearOpMode {

    // 목표 heading (라디안). 기본 +90도(CCW).
    private static final double TARGET_HEADING = Math.toRadians(90);

    // 회전 제어 부호. +1 로 먼저 시도, 폭주하면 -1로 바꿔 재검증.
    private static final double SIGN = +1.0;

    // 실기 검증: 게인을 낮춰도 목표 근처에서 진동 지속 → 정지마찰(static friction) 데드존이
    // 원인으로 판단. 오차가 작아지면 출력도 작아지는데, 그 출력이 모터를 움직이기엔 부족해서
    // 안 움직이다가 오차가 더 쌓이면 갑자기 튀는 현상. MIN_TURN_POWER로 최소 출력을 보장.
    // 실기 검증: 오차가 클 때(180도 근처) MAX_TURN=0.18이 마찰을 이기기엔 약해서
    // 모터가 나아가지 못하고 제자리서 버징(시동-실속 반복)함. 최대출력을 확실히 올림.
    // 목표 근처는 latch/MIN_TURN_POWER가 별도로 부드럽게 처리하므로 안전함.
    private static final double KP_HEADING = 0.6;
    private static final double MAX_TURN = 0.45;
    private static final double MIN_TURN_POWER = 0.15; // 이보다 작은 출력은 0으로(허용오차 안) or 최소값으로 올림
    // 실기 검증(90도 목표): 래치까지 넣어도 IMU노이즈/기계 유격으로 소폭(±수도) 잔떨림 존재.
    // 실사용엔 문제없는 수준이라 허용오차를 넉넉히 키워 안정화.
    private static final double HEADING_TOLERANCE = Math.toRadians(8);
    // 래치(이력현상): 한 번 도달하면, 오차가 이 임계값을 넘을 때까지 재구동 안 함.
    private static final double RESUME_THRESHOLD = Math.toRadians(14);
    // 실기 검증: 오차가 ±180° 경계 근처면 좌우 어느 쪽이 가깝나 판단이 노이즈로 계속 뒤집혀
    // 회전 방향이 버벅임(방향 미결정). 한 번 회전 방향을 정하면(DIR_LOCK_DEG 벗어난 뒤)
    // 도달할 때까지 그 방향을 고정해 해결.
    private static final double DIR_LOCK_DEG = 160.0; // 이 각도 넘게 벗어나면 그때 방향 고정

    @Override
    public void runOpMode() {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        Localizer loc = new Localizer(hardwareMap, "perpEncoder", "parallelEncoder");
        drive.setOpenLoop();

        telemetry.addLine("제자리 회전 검증. 목표 +90° (CCW)");
        telemetry.addLine("SIGN=" + SIGN + " (폭주하면 STOP 후 -1로 바꿔 재검증)");
        telemetry.addLine("START 후 회전. 반대로 돌거나 폭주하면 즉시 STOP!");
        telemetry.update();

        waitForStart();
        loc.resetPose();

        boolean latched = false;   // 도달 후 정지 유지 상태
        double dirLock = 0.0;      // 0=미결정, +1/-1=고정된 회전 방향

        while (opModeIsActive()) {
            loc.update();
            double th = loc.getHeading();

            double eHeading = normalizeAngle(TARGET_HEADING - th);
            double absErr = Math.abs(eHeading);

            // 방향 고정: 오차가 DIR_LOCK_DEG를 넘게 벗어나면 그 시점의 부호로 방향을 고정.
            // 경계(±180) 근처 노이즈로 좌우가 뒤집히는 걸 막는다.
            if (Math.toDegrees(absErr) > DIR_LOCK_DEG) {
                dirLock = Math.signum(eHeading);
            }

            // 래치 갱신: 도달하면 걸고, 오차가 RESUME_THRESHOLD 넘어야 다시 풀림
            if (!latched && absErr < HEADING_TOLERANCE) {
                latched = true;
                dirLock = 0.0; // 도달했으니 방향고정 해제(다음 회전 때 새로 판단)
            } else if (latched && absErr > RESUME_THRESHOLD) {
                latched = false;
            }

            double w = 0.0;
            if (!latched) {
                // 방향이 고정돼 있으면 부호를 그 방향으로 강제, 크기는 오차 비례 그대로
                double signedErr = (dirLock != 0.0) ? dirLock * absErr : eHeading;
                w = clamp(SIGN * KP_HEADING * signedErr, -MAX_TURN, MAX_TURN);
                // 실기 검증: 목표 근처(RESUME_THRESHOLD 이내)에서 최소출력 강제하면 매번
                // 오버슈트→좌우진동 발생. 이 구간에선 최소출력 강제 안 하고 자연 감쇠시킴.
                if (absErr <= RESUME_THRESHOLD) {
                    // 그대로 둠 (min power 미적용) — 못 움직이면 latch가 곧 반경 안으로 판정
                } else if (Math.abs(w) < MIN_TURN_POWER) {
                    w = Math.copySign(MIN_TURN_POWER, w);
                }
            }

            drive.driveRobotRelative(0, 0, w);  // 제자리 회전만
            boolean reached = latched;

            telemetry.addData("현재 heading(°)", "%.1f", Math.toDegrees(th));
            telemetry.addData("목표 heading(°)", "%.1f", Math.toDegrees(TARGET_HEADING));
            telemetry.addData("오차(°)", "%.1f", Math.toDegrees(eHeading));
            telemetry.addData("w 출력", "%.2f", w);
            telemetry.addData("상태", reached ? "도달! (부호 맞음)" : "회전중...");
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
