package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * 오도메트리 회전 진단 + 모터기반 오프셋 재측정 (연결/대시보드 불필요, DS 화면만 봄).
 *
 * 목적: goto 회전이 이상한 게 "오도(하드웨어/오프셋)" 문제인지 "제어" 문제인지 확정한다.
 *   로봇을 모터로 "제자리에서 천천히" 계속 돌리면서 DS에 X, Y, heading, 실측 오프셋을 띄운다.
 *
 * 판정:
 *   - 도는 동안 X, Y가 0 근처 유지  → 오도 정상 → goto 문제는 제어/연결 쪽
 *   - X, Y가 슬금슬금 커짐          → 오도가 회전을 병진으로 오해 = 오프셋/하드웨어 문제
 *                                     → 화면의 PARALLEL_OFFSET / PERP_OFFSET (모터기반 실측)을
 *                                        Localizer 상수에 넣고 재배포하면 잡힘.
 *
 * ※ 손 회전으로 잰 오프셋이 모터 회전과 안 맞을 수 있어, 여기선 "실제 모터 회전"으로 다시 잰다.
 *
 * ⚠️ 안전: 바퀴 땅에 닿게, 주변 여유공간, 제자리 회전만(천천히). STOP 손 근처.
 */
@Autonomous(name = "Odo Spin Diagnostic", group = "Test")
public class OdoSpinDiagnostic extends LinearOpMode {

    // 진단용 느린 제자리 회전 출력 (안전하게 낮게). 부호는 방향만 결정, 무관.
    private static final double SPIN_POWER = 0.22;

    @Override
    public void runOpMode() {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        Localizer loc = new Localizer(hardwareMap, "perpEncoder", "parallelEncoder");
        drive.setOpenLoop();

        telemetry.addLine("오도 회전 진단 (제자리 천천히 회전)");
        telemetry.addLine("START 후 로봇이 천천히 돎. X·Y가 0 근처면 오도 정상.");
        telemetry.addLine("X·Y가 커지면 오프셋 문제 → 화면 오프셋값을 Localizer에 넣기.");
        telemetry.addLine("⚠️ 주변 여유공간, STOP 손 근처");
        telemetry.update();

        waitForStart();
        loc.resetPose();

        while (opModeIsActive()) {
            loc.update();

            // 제자리 회전만 (병진 0)
            drive.driveRobotRelative(0, 0, SPIN_POWER);

            double cumDeg = Math.toDegrees(loc.getCumHeading());
            telemetry.addData("── 오도가 튀나? (0 근처여야 정상) ──", "");
            telemetry.addData("X (m)", "%.3f", loc.getX());
            telemetry.addData("Y (m)", "%.3f", loc.getY());
            telemetry.addData("heading (°)", "%.1f", Math.toDegrees(loc.getHeading()));
            telemetry.addLine("── 모터기반 실측 오프셋 (안정되면 Localizer에 입력) ──");
            telemetry.addData("PARALLEL_OFFSET", "%.4f m", loc.calibParallelOffset());
            telemetry.addData("PERP_OFFSET", "%.4f m", loc.calibPerpOffset());
            telemetry.addData("누적 회전", "%.0f° (%.1f바퀴)", cumDeg, cumDeg / 360.0);
            telemetry.addLine("5바퀴+ 돈 뒤 값 읽고 STOP");
            telemetry.update();
        }

        drive.driveRobotRelative(0, 0, 0);
    }
}
