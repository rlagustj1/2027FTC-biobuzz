package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * DriveSubsystem을 이용해 네 모터를 각각 지정한 틱만큼 이동시키고,
 * 전부 목표 위치에 도달하면 끝나는 Command.
 *
 * CommandScheduler가 매 루프마다 isFinished()를 검사하는 방식(polling)이라,
 * EncoderMoveTest.java에서 쓰던 while(isBusy()) 블로킹 루프가 필요 없다.
 * 이 덕분에 여러 Subsystem을 동시에(병렬로) 움직이는 것도 SequentialCommandGroup /
 * ParallelCommandGroup으로 자유롭게 조합할 수 있다.
 */
public class DriveDistanceCommand extends CommandBase {

    private final DriveSubsystem drive;
    private final double lfTicks, rfTicks, lbTicks, rbTicks;
    private final double power;

    /**
     * @param drive   이 Command가 사용할 DriveSubsystem
     * @param lfTicks lf 모터 목표 이동량 (엔코더 틱)
     * @param rfTicks rf 모터 목표 이동량 (엔코더 틱)
     * @param lbTicks lb 모터 목표 이동량 (엔코더 틱)
     * @param rbTicks rb 모터 목표 이동량 (엔코더 틱)
     * @param power   모터 출력 배율 (0.0 ~ 1.0)
     */
    public DriveDistanceCommand(DriveSubsystem drive, double lfTicks, double rfTicks,
                                 double lbTicks, double rbTicks, double power) {
        this.drive = drive;
        this.lfTicks = lfTicks;
        this.rfTicks = rfTicks;
        this.lbTicks = lbTicks;
        this.rbTicks = rbTicks;
        this.power = power;
        addRequirements(drive); // 같은 Subsystem을 쓰는 다른 Command와 동시 실행 방지
    }

    @Override
    public void initialize() {
        drive.startMoveByTicks(lfTicks, rfTicks, lbTicks, rbTicks, power);
    }

    @Override
    public boolean isFinished() {
        return !drive.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
