package org.firstinspires.ftc.teamcode.localization;

/**
 * 메카넘 드라이브트레인의 야코비안 관련 계산 모음.
 * sim/jacobian_experiment.py 를 FTC SDK로 1:1 이식한 것.
 *
 * 야코비안 정의 (바퀴 접지 선속도 = J · 섀시속도):
 *   [r*ω_lf]   [1  -1  -L] [vx]
 *   [r*ω_rf] = [1   1   L] [vy]
 *   [r*ω_lb]   [1   1  -L] [wz]
 *   [r*ω_rb]   [1  -1   L]
 *
 * 이 4x3 과결정(바퀴 4 / 자유도 3) 구조 때문에:
 *   - 순기구학은 유사역행렬 J⁺ = (JᵀJ)⁻¹Jᵀ 로 최소제곱해를 구함
 *   - Jᵀ의 1차원 영공간 = 섀시를 안 움직이는 바퀴조합 = 슬립 검출 방향
 *
 * 단위: vx,vy [m/s], wz [rad/s], 바퀴 각속도 [rad/s].
 * 바퀴 순서는 항상 {lf, rf, lb, rb}.
 */
public class MecanumJacobian {

    // === 실측 필요 파라미터 (CLAUDE.md 규칙 1 — placeholder) ===================
    // TODO: 실측 필요 — 구동 바퀴 반지름 (goBILDA 96mm 메카넘 → 0.048m)
    private static final double WHEEL_RADIUS = 0.048; // m
    // TODO: 실측 필요 — 회전 레버암 L = 트랙폭/2 + 휠베이스/2 (실측)
    private static final double HALF_TRACK = 0.18;    // m
    private static final double HALF_BASE = 0.16;     // m
    private static final double L = HALF_TRACK + HALF_BASE;

    private static final double r = WHEEL_RADIUS;

    /**
     * 역기구학: 섀시 속도 -> 바퀴 각속도 (rad/s).
     * (JacobianExperiment.chassis_to_wheels 와 동일)
     *
     * @param vx 전방 속도 (m/s, +forward)
     * @param vy 좌측 속도 (m/s, +left)
     * @param wz 회전 각속도 (rad/s, +CCW)
     * @return {lf, rf, lb, rb} 바퀴 각속도 (rad/s)
     */
    public static double[] chassisToWheels(double vx, double vy, double wz) {
        double lf = (vx - vy - L * wz) / r;
        double rf = (vx + vy + L * wz) / r;
        double lb = (vx + vy - L * wz) / r;
        double rb = (vx - vy + L * wz) / r;
        return new double[]{lf, rf, lb, rb};
    }

    /**
     * 순기구학(유사역행렬): 바퀴 각속도 4개 -> 섀시 속도 3개, 최소제곱해.
     * JᵀJ = diag(4, 4, 4L²) 라서 J⁺ 를 손으로 닫힌형(closed-form)으로 전개할 수 있다.
     * (라이브러리 행렬연산 없이 계산 — Control Hub 루프에서 가볍게 돎)
     *
     * @param wheelOmega {lf, rf, lb, rb} 바퀴 각속도 (rad/s)
     * @return {vx, vy, wz} 섀시 속도 (m/s, m/s, rad/s)
     */
    public static double[] wheelsToChassis(double[] wheelOmega) {
        double lf = wheelOmega[0] * r; // 바퀴 선속도로 환산
        double rf = wheelOmega[1] * r;
        double lb = wheelOmega[2] * r;
        double rb = wheelOmega[3] * r;

        // J⁺ 닫힌형: JᵀJ = diag(4,4,4L²) 이용
        double vx = (lf + rf + lb + rb) / 4.0;
        double vy = (-lf + rf + lb - rb) / 4.0;
        double wz = (-lf + rf - lb + rb) / (4.0 * L);
        return new double[]{vx, vy, wz};
    }

    /**
     * 영공간 슬립 지표. 측정된 바퀴 각속도를 Jᵀ 영공간 방향 [1,1,-1,-1]/2 에 투영한 값.
     * 0에 가까우면 정상, 0에서 크게 벗어나면 앞바퀴 vs 뒷바퀴 스크럽(슬립) 신호.
     * (JacobianExperiment.slip_metric 와 동일)
     *
     * @param wheelOmega {lf, rf, lb, rb} 바퀴 각속도 (rad/s)
     * @return 슬립 지표 (rad/s)
     */
    public static double slipMetric(double[] wheelOmega) {
        // 영공간 단위벡터 [1,1,-1,-1]/2
        return 0.5 * (wheelOmega[0] + wheelOmega[1] - wheelOmega[2] - wheelOmega[3]);
    }

    private MecanumJacobian() {} // 유틸 클래스 — 인스턴스화 방지
}
