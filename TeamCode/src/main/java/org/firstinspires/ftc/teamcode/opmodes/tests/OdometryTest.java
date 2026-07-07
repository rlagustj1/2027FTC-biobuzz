package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.localization.Localizer;

/**
 * 오도메트리 검증용 OpMode.
 * 로봇을 손으로 밀면서 텔레메트리로 추정 pose(x, y, heading)와
 * 원시 엔코더 틱을 실시간 확인한다. 모터는 안 돌린다 (안전).
 *
 * 검증 방법:
 *   1. START 후 로봇을 손으로 밀기
 *   2. 앞으로 밀기      → X 증가해야 함 (parallel 틱 변화)
 *   3. 왼쪽으로 밀기    → Y 증가해야 함 (perp 틱 변화)
 *   4. 제자리 CCW 회전  → heading(θ) 증가, X/Y 거의 유지
 *   부호가 반대면 코드에서 방향 뒤집기.
 *
 * X 버튼 = 좌표 리셋(0,0).
 */
@TeleOp(name = "Odometry Test", group = "Test")
public class OdometryTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Config의 Expansion Hub 모터 이름과 일치해야 함.
        // ※ 실기 검증 결과 두 데드휠이 물리적으로 반대로 배치돼 있어 순서를 바꿔 넘김:
        //   config "perpEncoder"(포트1) = 실제 전진(parallel) 측정 휠
        //   config "parallelEncoder"(포트0) = 실제 스트레이프(perp) 측정 휠
        Localizer loc = new Localizer(hardwareMap, "perpEncoder", "parallelEncoder");

        telemetry.addLine("오도메트리 검증. START 후 로봇을 손으로 밀어보세요.");
        telemetry.addLine("앞으로→X+ / 왼쪽→Y+ / CCW회전→θ+ 확인");
        telemetry.addLine("X 버튼 = 좌표 리셋");
        telemetry.update();

        waitForStart();
        loc.resetPose();

        while (opModeIsActive()) {
            loc.update();

            if (gamepad1.x) {
                loc.resetPose();
            }

            telemetry.addLine(loc.debugString());
            telemetry.addLine("--- 원시값 (방향 검증용) ---");
            telemetry.addData("parallel 틱", loc.getParallelTicks());
            telemetry.addData("perp 틱", loc.getPerpTicks());
            telemetry.addData("IMU heading(°)", "%.1f", Math.toDegrees(loc.getImuHeading()));
            telemetry.addLine("--- 추정 pose ---");
            telemetry.addData("X (m, 전방+)", "%.3f", loc.getX());
            telemetry.addData("Y (m, 좌측+)", "%.3f", loc.getY());
            telemetry.addData("heading (°)", "%.1f", Math.toDegrees(loc.getHeading()));
            telemetry.update();
        }
    }
}
