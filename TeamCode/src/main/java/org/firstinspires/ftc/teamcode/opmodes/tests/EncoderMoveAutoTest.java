package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.commands.DriveDistanceCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * 게임패드 없이 START만 눌러도 자동으로
 * 전진 1바퀴 -> 후진 1바퀴 -> 좌 스트레이프 1바퀴 -> 우 스트레이프 1바퀴 순서로
 * 실행되는 엔코더 이동 테스트용 Autonomous OpMode.
 *
 * CommandOpMode 구조: initialize()에서 "무엇을 할지"(Command 조합)만 등록하고,
 * 실제 반복 실행/종료 판정은 FTCLib의 CommandScheduler가 대신 해준다.
 */
@Autonomous(name = "Encoder Move Auto Test", group = "Test")
public class EncoderMoveAutoTest extends CommandOpMode {

    // goBILDA 5203 series, 312 RPM (19.2:1) 모델 데이터시트 기준 이론값.
    // TODO: 실측 권장 — 배터리 전압/부하 상태에 따라 약간의 오차가 있을 수 있으니
    // 실제로 1바퀴 돌려보고 로봇이 정확히 1회전 하는지 확인할 것.
    private static final double TICKS_PER_REV = 537.7; // 단위: 엔코더 틱 / 모터축(출력축) 1회전

    private static final double DRIVE_POWER = 0.4;

    @Override
    public void initialize() {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);

        schedule(new SequentialCommandGroup(
                // 전진 1바퀴
                new DriveDistanceCommand(drive, TICKS_PER_REV, TICKS_PER_REV, TICKS_PER_REV, TICKS_PER_REV, DRIVE_POWER),
                new WaitCommand(1000),
                // 후진 1바퀴
                new DriveDistanceCommand(drive, -TICKS_PER_REV, -TICKS_PER_REV, -TICKS_PER_REV, -TICKS_PER_REV, DRIVE_POWER),
                new WaitCommand(1000),
                // 좌 스트레이프 1바퀴 (X자 메카넘 배치 기준: lf,rb 후진 / rf,lb 전진)
                new DriveDistanceCommand(drive, -TICKS_PER_REV, TICKS_PER_REV, TICKS_PER_REV, -TICKS_PER_REV, DRIVE_POWER),
                new WaitCommand(1000),
                // 우 스트레이프 1바퀴 (좌측의 반대)
                new DriveDistanceCommand(drive, TICKS_PER_REV, -TICKS_PER_REV, -TICKS_PER_REV, TICKS_PER_REV, DRIVE_POWER),
                new WaitCommand(1000)
        ));
    }
}
