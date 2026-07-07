package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.localization.MecanumJacobian;
import org.junit.Test;

/**
 * MecanumJacobian 로직이 sim/jacobian_experiment.py 의 검증 결과와 일치하는지
 * 확인하는 JVM 단위 테스트 (실물 로봇 불필요, CLAUDE.md 규칙 4).
 */
public class MecanumJacobianTest {

    private static final double EPS = 1e-9;

    /** ① 역기구학 -> 순기구학 왕복: 슬립 없으면 원래 섀시 속도 복원돼야 함. */
    @Test
    public void roundTripRecoversChassisVelocity() {
        double vx = 0.7, vy = -0.3, wz = 1.2;
        double[] wheels = MecanumJacobian.chassisToWheels(vx, vy, wz);
        double[] recovered = MecanumJacobian.wheelsToChassis(wheels);

        assertEquals(vx, recovered[0], EPS);
        assertEquals(vy, recovered[1], EPS);
        assertEquals(wz, recovered[2], EPS);
    }

    /** 순수 전진 명령이면 4바퀴 각속도가 모두 같아야 함. */
    @Test
    public void pureForwardGivesEqualWheels() {
        double[] w = MecanumJacobian.chassisToWheels(1.0, 0.0, 0.0);
        assertEquals(w[0], w[1], EPS);
        assertEquals(w[1], w[2], EPS);
        assertEquals(w[2], w[3], EPS);
    }

    /** ② 정상 주행이면 슬립 지표 ≈ 0. */
    @Test
    public void cleanDrivingHasZeroSlip() {
        double[] clean = MecanumJacobian.chassisToWheels(0.6, 0.0, 0.0);
        assertEquals(0.0, MecanumJacobian.slipMetric(clean), EPS);
    }

    /** ② 앞바퀴만 슬립나면 슬립 지표가 0에서 벗어나야 함. */
    @Test
    public void frontWheelSlipIsDetected() {
        double[] slipped = MecanumJacobian.chassisToWheels(0.6, 0.0, 0.0);
        slipped[0] *= 1.2; // lf 20% 헛돎
        slipped[1] *= 1.2; // rf 20% 헛돎
        assertTrue("앞바퀴 슬립이 감지돼야 함", Math.abs(MecanumJacobian.slipMetric(slipped)) > 0.1);
    }

    /** 순수 스트레이프(+left): lf,rb 음 / rf,lb 양 (부호 패턴 확인). */
    @Test
    public void pureStrafeSignPattern() {
        double[] w = MecanumJacobian.chassisToWheels(0.0, 1.0, 0.0);
        assertTrue(w[0] < 0); // lf
        assertTrue(w[1] > 0); // rf
        assertTrue(w[2] > 0); // lb
        assertTrue(w[3] < 0); // rb
    }
}
