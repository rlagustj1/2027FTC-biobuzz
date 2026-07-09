package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.localization.Localizer;

/**
 * 오도메트리 데드휠 오프셋(PARALLEL_OFFSET, PERP_OFFSET) 실측 캘리브레이션.
 *
 * 문제: 데드휠이 로봇 회전중심에서 떨어져 장착돼 있어, 제자리 회전 시 데드휠이 원호를 그리며
 *   엔코더가 돌아간다. 이걸 병진으로 오해하면(offset=0) 회전할 때마다 위치추정에 phantom
 *   이동이 섞여 goto가 발산한다. (실측 예: 순수 회전 명령인데 Y가 ~1m 튐)
 *
 * 원리: 제자리 순수 회전이면 실제 병진은 0이므로
 *   Σ dPar  = PARALLEL_OFFSET * Σ dHeading   →  PARALLEL_OFFSET = Σ dPar / Σ dHeading
 *   Σ dPerp = PERP_OFFSET     * Σ dHeading   →  PERP_OFFSET     = Σ dPerp / Σ dHeading
 *
 * 사용법 (모터 안 씀, 안전):
 *   1. 로봇을 바닥에 두고 START.
 *   2. 로봇을 손으로 제자리에서 "여러 바퀴"(예: 5~10바퀴) 천천히 돌린다.
 *      (많이 돌릴수록 노이즈가 줄어 정확해짐. 제자리 회전이 핵심 — 옆으로 밀지 말 것.)
 *   3. 화면의 PARALLEL_OFFSET / PERP_OFFSET 값이 안정되면 그 값을 읽는다.
 *   4. 그 값을 Localizer.java 의 PARALLEL_OFFSET / PERP_OFFSET 상수에 넣고 재배포.
 *
 * ⚠️ 제자리 회전이 정확해야 한다(로봇 중심이 안 움직이게 그 자리서 회전). 병진이 섞이면 오차.
 */
@Autonomous(name = "Odo Offset Calibration", group = "Test")
public class OdoOffsetCalibration extends LinearOpMode {

    @Override
    public void runOpMode() {
        // 데드휠 물리 배치가 반대라 인자 스왑 (다른 OpMode와 동일)
        Localizer loc = new Localizer(hardwareMap, "perpEncoder", "parallelEncoder");

        telemetry.addLine("오도 오프셋 캘리브레이션");
        telemetry.addLine("START 후 로봇을 손으로 '제자리에서' 여러 바퀴 돌리세요.");
        telemetry.addLine("(옆으로 밀지 말 것 — 순수 회전만. 많이 돌릴수록 정확)");
        telemetry.update();

        waitForStart();
        loc.resetPose();

        while (opModeIsActive()) {
            loc.update();

            double cumDeg = Math.toDegrees(loc.getCumHeading());
            double parOff = loc.calibParallelOffset();
            double perpOff = loc.calibPerpOffset();

            telemetry.addData("누적 회전", "%.0f° (%.1f 바퀴)", cumDeg, cumDeg / 360.0);
            telemetry.addLine("── 아래 값을 Localizer 상수에 입력 ──");
            telemetry.addData("PARALLEL_OFFSET", "%.4f m", parOff);
            telemetry.addData("PERP_OFFSET", "%.4f m", perpOff);
            telemetry.addLine("");
            telemetry.addData("현재 pose", "X=%.3f Y=%.3f θ=%.1f°",
                    loc.getX(), loc.getY(), Math.toDegrees(loc.getHeading()));
            telemetry.addLine("충분히(5바퀴+) 돌려 값이 안정되면 기록 후 STOP");
            telemetry.update();
        }
    }
}
