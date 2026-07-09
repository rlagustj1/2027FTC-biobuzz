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

        // 실기 확정(MotorMappingTest, 재배선 후): +power에서 왼쪽(lf,lb)이 뒤로, 오른쪽(rf,rb)이
        // 앞으로 돌았음. 왼쪽을 REVERSE로 잡아 네 바퀴 모두 정방향=전진이 되게 함.
        // (메카넘은 좌우 미러 장착이라 한쪽 반전이 표준)
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);

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

    /** 개루프 주행 모드로 전환 (경로추종용). 위치는 데드휠 오도로 잰다. */
    public void setOpenLoop() {
        for (DcMotorEx motor : new DcMotorEx[]{lf, rf, lb, rb}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * 로봇 로컬 프레임 섀시 속도 명령으로 바퀴 출력을 설정 (개루프).
     * 메카넘 역기구학(정규화된 형태) 그대로 사용. sim mecanum_inverse_kinematics와 동일 부호.
     *
     * @param vx 전방 성분 (+forward), 대략 -1~1
     * @param vy 좌측 성분 (+left)
     * @param w  회전 성분 (+CCW)
     */
    public void driveRobotRelative(double vx, double vy, double w) {
        double lfP = vx - vy - w;
        double rfP = vx + vy + w;
        double lbP = vx + vy - w;
        double rbP = vx - vy + w;

        // 최댓값이 1을 넘으면 전체를 비례 축소 (방향 유지하며 포화 방지)
        double max = Math.max(1.0, Math.max(Math.abs(lfP),
                Math.max(Math.abs(rfP), Math.max(Math.abs(lbP), Math.abs(rbP)))));
        lf.setPower(lfP / max);
        rf.setPower(rfP / max);
        lb.setPower(lbP / max);
        rb.setPower(rbP / max);
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
