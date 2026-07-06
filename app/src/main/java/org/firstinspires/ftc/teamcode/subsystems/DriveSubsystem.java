package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * 메카넘 드라이브트레인 하드웨어(모터 4개)만 담당하는 Subsystem.
 * "어떻게 움직일지"는 여기 없고, 오직 "모터를 어떻게 제어하는지"만 안다.
 * 실제 이동 동작(몇 틱 갈지, 언제 멈출지)은 Command 쪽 책임이다.
 */
public class DriveSubsystem extends SubsystemBase {

    private final DcMotorEx lf, rf, lb, rb;

    public DriveSubsystem(HardwareMap hardwareMap) {
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        // TODO: 실기 확인 필요 — 전진 명령 시 로봇이 실제로 앞으로 가는지 보고
        // 방향이 반대면 여기서 FORWARD/REVERSE를 바꿀 것 (메카넘은 한쪽 반전이 표준).
        lf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotorEx motor : new DcMotorEx[]{lf, rf, lb, rb}) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * 네 모터를 각각 지정한 틱만큼(현재 위치 기준 상대값) 목표를 설정하고
     * RUN_TO_POSITION 모드로 지정한 출력으로 이동을 시작한다. 논블로킹 — 도달 여부는
     * {@link #isBusy()}로 별도 확인해야 한다.
     *
     * @param lfTicks lf 모터 목표 이동량 (엔코더 틱)
     * @param rfTicks rf 모터 목표 이동량 (엔코더 틱)
     * @param lbTicks lb 모터 목표 이동량 (엔코더 틱)
     * @param rbTicks rb 모터 목표 이동량 (엔코더 틱)
     * @param power   모터 출력 배율 (0.0 ~ 1.0)
     */
    public void startMoveByTicks(double lfTicks, double rfTicks, double lbTicks, double rbTicks, double power) {
        lf.setTargetPosition(lf.getCurrentPosition() + (int) Math.round(lfTicks));
        rf.setTargetPosition(rf.getCurrentPosition() + (int) Math.round(rfTicks));
        lb.setTargetPosition(lb.getCurrentPosition() + (int) Math.round(lbTicks));
        rb.setTargetPosition(rb.getCurrentPosition() + (int) Math.round(rbTicks));

        for (DcMotorEx motor : new DcMotorEx[]{lf, rf, lb, rb}) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }
    }

    /** 네 모터 중 하나라도 목표 위치에 도달하지 못했으면 true. */
    public boolean isBusy() {
        return lf.isBusy() || rf.isBusy() || lb.isBusy() || rb.isBusy();
    }

    /** 모터 출력을 0으로 하고 다음 명령을 받을 수 있는 상태(RUN_USING_ENCODER)로 되돌린다. */
    public void stop() {
        for (DcMotorEx motor : new DcMotorEx[]{lf, rf, lb, rb}) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /** 텔레메트리 출력용 현재 엔코더 위치 (엔코더 틱). 순서: lf, rf, lb, rb. */
    public int[] getPositions() {
        return new int[]{lf.getCurrentPosition(), rf.getCurrentPosition(), lb.getCurrentPosition(), rb.getCurrentPosition()};
    }
}
