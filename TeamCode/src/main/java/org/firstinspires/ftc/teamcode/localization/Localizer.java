package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * 2륜 데드휠 + IMU 오도메트리.
 * sim/odometry_ik_sim.py 의 TwoWheelOdometry 클래스를 FTC SDK로 1:1 이식한 것.
 *
 * 좌표계 (시뮬과 동일):
 *   필드 X = 전방(+forward), Y = 좌측(+left), heading θ = +CCW.
 *   단위: 위치 m, 각도 rad.
 *
 * 원리:
 *   - parallel 데드휠: 로봇 로컬 x(전진) 이동 측정
 *   - perpendicular 데드휠: 로컬 y(스트레이프) 이동 측정 (단, 회전 성분 보정 필요)
 *   - heading: IMU에서 직접 읽음 (데드휠 각도 적분 오차 누적을 피함)
 *
 * 하드웨어 결선(사용자 확정): 오도휠 엔코더 2개를 구동모터 엔코더 포트에 결선한다.
 *   따라서 엔코더 값은 해당 포트의 DcMotorEx.getCurrentPosition()으로 읽는다.
 *   (모터 구동과 엔코더 소스가 물리적으로 다른 것에 주의 — 방향/부호 실측 확인 필수)
 */
public class Localizer {

    // === goBILDA 오도메트리 팟 기준값 (일부 실측 권장) ==========================
    // goBILDA Odometry Pod: 휠 지름 48mm → 반지름 0.024m
    private static final double DEAD_WHEEL_RADIUS = 0.024; // m
    // 실측 보정 (규칙 7): 1m 반복 측정으로 수렴. CPR 2619→1.1 나와 2881로 재보정.
    //   손push 특성상 ±5~10% 노이즈 존재. 실제 모터 주행 시 더 정확.
    private static final double ODO_TICKS_PER_REV = 2881;  // 틱/회전 (실측 보정값)
    // 데드휠 회전 방향 부호 (장착 방향에 따라 결정, 실측으로 확정).
    // 실측: 앞으로 미는데 X가 음수로 나옴 → parallel 부호 뒤집음(-1).
    private static final double PARALLEL_DIR = -1.0;
    private static final double PERP_DIR = +1.0;           // 좌측→Y+ 정상이라 유지
    // TODO: 실측 필요 — 데드휠 장착 오프셋 (로봇 중심 기준, 방향검증 후 정밀 측정)
    //   PARALLEL_OFFSET: 전진 휠이 중심에서 Y로 떨어진 거리 (m)
    //   PERP_OFFSET:     스트레이프 휠이 중심에서 X로 떨어진 거리 (m)
    //   ※ 오프셋은 "제자리 회전 검증" 단계에서 실측해 넣을 것 (지금은 임시 0으로 시작)
    private static final double PARALLEL_OFFSET = 0.0;     // m (임시 — 회전검증 후 실측)
    private static final double PERP_OFFSET = 0.0;         // m (임시 — 회전검증 후 실측)

    // 파생값: 오도휠 1틱당 이동 거리 (m/틱)
    private static final double ODO_M_PER_TICK =
            (2.0 * Math.PI * DEAD_WHEEL_RADIUS) / ODO_TICKS_PER_REV;

    // === 하드웨어 ============================================================
    private final DcMotorEx parallelEncoder; // 전진 측정 데드휠 (구동모터 포트에서 읽음)
    private final DcMotorEx perpEncoder;      // 스트레이프 측정 데드휠
    private final IMU imu;

    // === 상태 (필드 좌표 추정) ================================================
    private double xM = 0.0;       // 필드 X (m)
    private double yM = 0.0;       // 필드 Y (m)
    private double headingRad = 0.0;
    private double prevHeadingRad = 0.0;
    private int prevParallelTicks = 0;
    private int prevPerpTicks = 0;

    /**
     * @param hardwareMap       OpMode의 hardwareMap
     * @param parallelEncoderName parallel 데드휠 엔코더가 결선된 "모터 포트" config 이름
     *                            (TODO: 실제 결선한 포트 이름으로 교체)
     * @param perpEncoderName     perpendicular 데드휠 엔코더가 결선된 config 이름
     */
    public Localizer(HardwareMap hardwareMap, String parallelEncoderName, String perpEncoderName) {
        parallelEncoder = hardwareMap.get(DcMotorEx.class, parallelEncoderName);
        perpEncoder = hardwareMap.get(DcMotorEx.class, perpEncoderName);

        imu = hardwareMap.get(IMU.class, "imu");
        // TODO: 실측 필요 — Control Hub의 실제 장착 방향으로 교체할 것.
        // (로고 방향/USB 포트 방향에 따라 LogoFacingDirection, UsbFacingDirection 결정)
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        resetEncoderBaseline();
    }

    /** 현재 엔코더/heading을 기준점으로 저장 (좌표는 유지). start 직후 호출 권장. */
    public void resetEncoderBaseline() {
        prevParallelTicks = parallelEncoder.getCurrentPosition();
        prevPerpTicks = perpEncoder.getCurrentPosition();
        prevHeadingRad = readImuHeading();
    }

    /** IMU에서 절대 heading(rad, +CCW) 읽기. */
    private double readImuHeading() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.RADIANS);
    }

    /**
     * 매 제어 루프마다 1회 호출. 엔코더/IMU 변화량으로 필드 좌표 추정을 갱신한다.
     * (sim TwoWheelOdometry.update() 와 동일한 수식)
     */
    public void update() {
        int parTicks = parallelEncoder.getCurrentPosition();
        int perpTicks = perpEncoder.getCurrentPosition();
        double heading = readImuHeading();

        // heading 변화량 (랩어라운드 -pi~pi 정규화)
        double dHeading = normalizeAngle(heading - prevHeadingRad);

        // 틱 변화량 -> 거리 (m), 장착 방향 부호 적용
        double dPar = (parTicks - prevParallelTicks) * ODO_M_PER_TICK * PARALLEL_DIR;
        double dPerp = (perpTicks - prevPerpTicks) * ODO_M_PER_TICK * PERP_DIR;

        // 회전에 의한 데드휠 회전분 보정 -> 순수 로컬 병진 성분
        double localDx = dPar - PARALLEL_OFFSET * dHeading;
        double localDy = dPerp - PERP_OFFSET * dHeading;

        // 이동 구간 중앙 heading으로 로컬->필드 변환 (1차 근사)
        double thMid = prevHeadingRad + dHeading / 2.0;
        xM += localDx * Math.cos(thMid) - localDy * Math.sin(thMid);
        yM += localDx * Math.sin(thMid) + localDy * Math.cos(thMid);
        headingRad = heading;

        // 다음 루프용 기준 갱신
        prevParallelTicks = parTicks;
        prevPerpTicks = perpTicks;
        prevHeadingRad = heading;
    }

    /** -pi ~ pi 로 각도 정규화. */
    private static double normalizeAngle(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    // === 추정 pose 접근자 (단위: m, m, rad) ===================================
    public double getX() { return xM; }
    public double getY() { return yM; }
    public double getHeading() { return headingRad; }

    // === 디버그용 원시값 접근자 (방향/부호 검증에 사용) =========================
    /** parallel 데드휠 현재 엔코더 틱 (원시). */
    public int getParallelTicks() { return parallelEncoder.getCurrentPosition(); }
    /** perpendicular 데드휠 현재 엔코더 틱 (원시). */
    public int getPerpTicks() { return perpEncoder.getCurrentPosition(); }
    /** IMU 원시 heading (rad). */
    public double getImuHeading() { return readImuHeading(); }

    /** 좌표를 (0,0,현재heading)으로 리셋. 검증 시작 전 호출. */
    public void resetPose() {
        xM = 0; yM = 0;
        resetEncoderBaseline();
    }

    /** 텔레메트리/디버그용 문자열. */
    public String debugString() {
        return String.format("Pose  X=%.3f m  Y=%.3f m  θ=%.1f°",
                xM, yM, Math.toDegrees(headingRad));
    }
}
