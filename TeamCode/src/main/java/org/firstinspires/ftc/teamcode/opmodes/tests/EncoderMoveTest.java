package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * 메카넘 휠 4개를 엔코더 기준 정확히 1바퀴 회전만큼 움직여
 * 전진/후진/좌우 스트레이프가 의도대로 동작하는지 확인하는 테스트용 OpMode.
 * 하드웨어 제어는 전부 {@link DriveSubsystem}에 위임하고, 이 클래스는
 * "어떤 버튼에 어떤 동작을 연결할지"만 담당한다.
 *
 * 게임패드 버튼:
 *   Y = 전진 1바퀴, A = 후진 1바퀴, X = 좌측 스트레이프 1바퀴, B = 우측 스트레이프 1바퀴
 */
@TeleOp(name = "Encoder Move Test", group = "Test")
public class EncoderMoveTest extends LinearOpMode {

    // goBILDA 5203 series, 312 RPM (19.2:1) 모델 데이터시트 기준 이론값.
    // TODO: 실측 권장 — 배터리 전압/부하 상태에 따라 약간의 오차가 있을 수 있으니
    // 실제로 1바퀴 돌려보고 로봇이 정확히 1회전 하는지 확인할 것.
    private static final double TICKS_PER_REV = 537.7; // 단위: 엔코더 틱 / 모터축(출력축) 1회전

    private static final double DRIVE_POWER = 0.4;

    private DriveSubsystem drive;

    @Override
    public void runOpMode() {
        drive = new DriveSubsystem(hardwareMap);

        telemetry.addLine("초기화 완료. Y=전진 A=후진 X=좌스트레이프 B=우스트레이프");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.y) {
                moveAndWait(TICKS_PER_REV, TICKS_PER_REV, TICKS_PER_REV, TICKS_PER_REV); // 전진
            } else if (gamepad1.a) {
                moveAndWait(-TICKS_PER_REV, -TICKS_PER_REV, -TICKS_PER_REV, -TICKS_PER_REV); // 후진
            } else if (gamepad1.x) {
                // 좌측 스트레이프: lf,rb 후진 / rf,lb 전진 (X자 메카넘 배치 기준)
                moveAndWait(-TICKS_PER_REV, TICKS_PER_REV, TICKS_PER_REV, -TICKS_PER_REV);
            } else if (gamepad1.b) {
                // 우측 스트레이프: 좌측의 반대
                moveAndWait(TICKS_PER_REV, -TICKS_PER_REV, -TICKS_PER_REV, TICKS_PER_REV);
            }

            int[] pos = drive.getPositions();
            telemetry.addData("lf pos", pos[0]);
            telemetry.addData("rf pos", pos[1]);
            telemetry.addData("lb pos", pos[2]);
            telemetry.addData("rb pos", pos[3]);
            telemetry.addData("busy", drive.isBusy());
            telemetry.update();
        }
    }

    /**
     * DriveSubsystem에 이동을 지시하고, 목표 위치 도달까지 대기(블로킹)한다.
     * 버튼 하나로 한 동작만 끝까지 실행하는 단순 테스트 용도라 블로킹으로 충분하다.
     */
    private void moveAndWait(double lfTicks, double rfTicks, double lbTicks, double rbTicks) {
        drive.startMoveByTicks(lfTicks, rfTicks, lbTicks, rbTicks, DRIVE_POWER);

        while (opModeIsActive() && drive.isBusy()) {
            telemetry.addLine("이동 중...");
            telemetry.update();
        }

        drive.stop();
    }
}
