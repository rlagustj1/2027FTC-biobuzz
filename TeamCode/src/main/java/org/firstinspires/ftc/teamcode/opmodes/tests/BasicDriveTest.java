package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * 메카넘 기본 3동작 검증 (방향 수정 후 확인용). 연결 불필요.
 *
 * 순서대로 각 2초씩: 전진 → (정지) → 왼쪽 옆이동 → (정지) → 제자리 회전(CCW).
 *
 * 판정:
 *   - 전진: 곧게 앞으로 (회전·치우침 없이)
 *   - 옆이동: 곧게 왼쪽으로 (회전 없이!) ← 여기가 핵심. 돌면 롤러 방향(X-config) 문제
 *   - 회전: 제자리에서 반시계(CCW)
 *
 * ⚠️ 안전: 넓은 공간, 저속(0.3), STOP 손 근처.
 */
@Autonomous(name = "Basic Drive Test", group = "Test")
public class BasicDriveTest extends LinearOpMode {

    private static final double P = 0.25;
    private static final long STEP_MS = 1000;

    @Override
    public void runOpMode() {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        drive.setOpenLoop();

        telemetry.addLine("메카넘 기본동작 검증");
        telemetry.addLine("전진 → 왼쪽옆이동 → 제자리회전, 각 2초");
        telemetry.addLine("특히 '옆이동'이 회전 없이 곧게 왼쪽으로 가는지 보세요");
        telemetry.addLine("⚠️ 넓은 공간, STOP 손 근처");
        telemetry.update();
        waitForStart();

        step(drive, P, 0, 0, "① 전진 (곧게 앞으로?)");
        pause(drive);
        step(drive, 0, P, 0, "② 왼쪽 옆이동 (회전없이 곧게 왼쪽?)");
        pause(drive);
        step(drive, 0, 0, P, "③ 제자리 회전 (CCW, 제자리?)");
        pause(drive);

        drive.driveRobotRelative(0, 0, 0);
        telemetry.addLine("끝. 각 동작이 의도대로였는지 기록하세요.");
        telemetry.update();
        sleep(2000);
    }

    private void step(DriveSubsystem drive, double vx, double vy, double w, String label) {
        long t0 = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - t0 < STEP_MS) {
            drive.driveRobotRelative(vx, vy, w);
            telemetry.addLine(">>> " + label);
            telemetry.addData("명령", "vx=%.1f vy=%.1f w=%.1f", vx, vy, w);
            telemetry.update();
            sleep(40);
        }
        drive.driveRobotRelative(0, 0, 0);
    }

    private void pause(DriveSubsystem drive) {
        drive.driveRobotRelative(0, 0, 0);
        sleep(800);
    }
}
