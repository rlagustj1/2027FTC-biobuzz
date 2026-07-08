"""
부록 A 실험: 로봇팔(2관절 평면팔)의 자세의존 야코비안과 특이점(Singularity)

목적 (세특 탐구 논지의 '대조축'):
  메카넘 드라이브(sim/jacobian_experiment.py)의 야코비안은 상수·과결정(4x3)이라
  특이점이 없다. 반대로 로봇팔의 야코비안은 자세에 따라 변하고(자세의존, 2x2),
  det(J)=0 이 되는 자세에서 특이점이 생겨 자유도를 순간적으로 잃는다.
  이 스크립트는 그 특이점을 직접 유도·시각화해서 "같은 야코비안 프레임에서
  팔은 특이점, 메카넘은 과결정"이라는 대조를 완성한다.

2관절 평면팔 순기구학 (링크길이 l1, l2 / 관절각 θ1, θ2):
  x = l1 cosθ1 + l2 cos(θ1+θ2)
  y = l1 sinθ1 + l2 sin(θ1+θ2)

위치 야코비안 (2x2):
  J = [ -l1 s1 - l2 s12 ,  -l2 s12 ]
      [  l1 c1 + l2 c12 ,   l2 c12 ]
  (s1=sinθ1, s12=sin(θ1+θ2), c1=cosθ1, c12=cos(θ1+θ2))

핵심 해석적 결과:
  det(J) = l1 * l2 * sin(θ2)
  → θ2 = 0 (팔 완전히 펴짐) 또는 θ2 = ±π (완전히 접힘) 에서 det=0 → 특이점.
  이때 말단은 '팔이 뻗은 방향(반경 방향)'으로 순간적으로 속도를 낼 수 없다.

단위: 길이 m, 각도 rad, 야코비안 성분 m/rad.
"""

import sys
import numpy as np
import matplotlib.pyplot as plt

try:
    sys.stdout.reconfigure(encoding="utf-8")
except AttributeError:
    pass

plt.rcParams["font.family"] = "Malgun Gothic"
plt.rcParams["axes.unicode_minus"] = False

# =============================================================================
# 링크 길이 (부록 A는 순수 이론/시뮬 — 실측 대상 아님. 대표값 사용)
#   실제 로봇팔이 아니라 '자코비안 특이점 개념'을 보이는 예시이므로 임의값 무방.
# =============================================================================
L1 = 1.0   # m (첫 번째 링크)
L2 = 0.8   # m (두 번째 링크)


def forward_kinematics(th1, th2):
    """관절각 -> 말단 위치 (x, y). 중간 관절 위치도 함께 반환(그림용)."""
    elbow = np.array([L1 * np.cos(th1), L1 * np.sin(th1)])
    end = elbow + np.array([L2 * np.cos(th1 + th2), L2 * np.sin(th1 + th2)])
    return elbow, end


def jacobian(th1, th2):
    """2x2 위치 야코비안 J(θ). 자세(θ1,θ2)에 따라 값이 변한다."""
    s1, c1 = np.sin(th1), np.cos(th1)
    s12, c12 = np.sin(th1 + th2), np.cos(th1 + th2)
    return np.array([
        [-L1 * s1 - L2 * s12, -L2 * s12],
        [ L1 * c1 + L2 * c12,  L2 * c12],
    ])


def det_analytic(th2):
    """해석적 행렬식: det(J) = l1*l2*sin(θ2). θ1에 무관."""
    return L1 * L2 * np.sin(th2)


def manipulability(th1, th2):
    """조작성 지수 w = sqrt(det(J Jᵀ)). 정사각 J에서는 |det J| 와 같다."""
    J = jacobian(th1, th2)
    # 특이점 근처에서 det(JJᵀ)가 부동소수 오차로 음수가 될 수 있어 0으로 클램프
    return float(np.sqrt(max(0.0, np.linalg.det(J @ J.T))))


# =============================================================================
# 검증 (규칙 4: assert)
# =============================================================================
def run_asserts():
    rng = np.random.default_rng(0)
    for _ in range(1000):
        th1 = rng.uniform(-np.pi, np.pi)
        th2 = rng.uniform(-np.pi, np.pi)
        J = jacobian(th1, th2)
        # (1) 수치 det 과 해석적 det = l1 l2 sinθ2 일치
        assert np.isclose(np.linalg.det(J), det_analytic(th2), atol=1e-12), \
            f"det 불일치 @({th1:.3f},{th2:.3f})"
        # (2) 조작성 w = |det J| (정사각)
        assert np.isclose(manipulability(th1, th2), abs(np.linalg.det(J)), atol=1e-9)

    # (3) 특이점: θ2=0, π 에서 det≈0 이고 rank 저하(<2)
    for th2_sing in (0.0, np.pi, -np.pi):
        J = jacobian(0.7, th2_sing)   # θ1 아무값
        assert abs(np.linalg.det(J)) < 1e-12, f"θ2={th2_sing} 특이점 아님"
        assert np.linalg.matrix_rank(J, tol=1e-9) < 2, f"θ2={th2_sing} rank 저하 아님"

    # (4) 비특이 자세는 rank 2 (full rank)
    assert np.linalg.matrix_rank(jacobian(0.7, 1.2), tol=1e-9) == 2

    print("[검증 통과] det=l1·l2·sinθ2 / w=|detJ| / θ2=0,±π 특이점(rank<2) / 일반자세 rank2")


# =============================================================================
# 특이점 근처에서 '말단이 못 내는 속도 방향'을 조작성 타원으로 보이기
#   타원의 두 반축 = J 의 특이값. 특이점에서 한 축이 0 → 타원이 '선'으로 붕괴.
# =============================================================================
def manipulability_axes(th1, th2):
    """조작성 타원의 두 축 벡터(방향*특이값) 반환."""
    J = jacobian(th1, th2)
    u, s, vt = np.linalg.svd(J)
    # 열: 작업공간(말단 속도) 주축 방향 u[:,i], 크기 s[i]
    return u, s


def plot_all():
    fig = plt.figure(figsize=(16, 5))

    # (1) det(J) vs θ2 : 0, ±π 에서 0을 지나는 사인 곡선 = 특이점 위치
    ax = fig.add_subplot(1, 3, 1)
    th2 = np.linspace(-np.pi, np.pi, 400)
    ax.plot(th2, det_analytic(th2), 'b-', lw=2)
    ax.axhline(0, color='gray', lw=0.8)
    for z in (-np.pi, 0.0, np.pi):
        ax.plot(z, 0, 'ro', ms=8)
    ax.annotate('특이점\n(팔 접힘 θ2=±π)', xy=(np.pi, 0), xytext=(1.2, 0.4),
                arrowprops=dict(arrowstyle='->', color='red'), color='red', fontsize=9)
    ax.annotate('특이점\n(팔 펴짐 θ2=0)', xy=(0, 0), xytext=(-2.6, 0.4),
                arrowprops=dict(arrowstyle='->', color='red'), color='red', fontsize=9)
    ax.set_xlabel('θ2 (rad)'); ax.set_ylabel('det J = l1·l2·sinθ2')
    ax.set_title('① 특이점 조건 det(J)=0\n(θ2=0: 완전히 펴짐, θ2=±π: 완전히 접힘)')
    ax.grid(True, alpha=0.3)

    # (2) 조작성 지수 히트맵 (θ1, θ2 평면): 특이점 근처에서 0으로 어두워짐
    ax = fig.add_subplot(1, 3, 2)
    g1 = np.linspace(-np.pi, np.pi, 200)
    g2 = np.linspace(-np.pi, np.pi, 200)
    G1, G2 = np.meshgrid(g1, g2)
    W = L1 * L2 * np.abs(np.sin(G2))   # w = |det J|
    im = ax.pcolormesh(G1, G2, W, shading='auto', cmap='viridis')
    ax.axhline(0, color='r', ls='--', lw=1)
    ax.axhline(np.pi, color='r', ls='--', lw=1)
    ax.axhline(-np.pi, color='r', ls='--', lw=1)
    fig.colorbar(im, ax=ax, label='조작성 w')
    ax.set_xlabel('θ1 (rad)'); ax.set_ylabel('θ2 (rad)')
    ax.set_title('② 조작성 지수 w(θ)\n빨간선(θ2=0,±π)=특이점, w→0')

    # (3) 팔 자세 + 조작성 타원: 일반자세(원만) vs 특이점(선으로 붕괴)
    ax = fig.add_subplot(1, 3, 3)
    ax.set_aspect('equal')
    configs = [
        (0.6, 1.3, 'tab:green', '일반 자세 (θ2=1.3)'),
        (0.6, 0.05, 'tab:red', '특이점 근처 (θ2=0 부근)'),
    ]
    scale = 0.25
    for th1, th2, col, lab in configs:
        elbow, end = forward_kinematics(th1, th2)
        ax.plot([0, elbow[0], end[0]], [0, elbow[1], end[1]], '-o', color=col, lw=2, label=lab)
        u, s, = manipulability_axes(th1, th2)
        # 타원: 말단(end)에 중심, 두 주축 u[:,0]*s0, u[:,1]*s1
        t = np.linspace(0, 2 * np.pi, 100)
        ell = (u[:, 0:1] * s[0] * np.cos(t) + u[:, 1:2] * s[1] * np.sin(t)) * scale
        ax.plot(end[0] + ell[0], end[1] + ell[1], '-', color=col, alpha=0.6)
    ax.plot(0, 0, 'ks', ms=8)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.set_title('③ 조작성 타원체\n일반=타원(2방향 가능) / 특이점=선으로 붕괴')
    ax.legend(fontsize=8, loc='lower left'); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = "sim/arm_singularity_result.png"
    plt.savefig(out, dpi=110)
    print(f"[저장] {out}")


if __name__ == "__main__":
    print(f"2관절 평면팔  L1={L1}m, L2={L2}m  (부록 A: 순수 이론 시뮬)")
    run_asserts()
    # 대표 특이/비특이 자세 조작성 출력
    print(f"  일반자세 θ2=1.3  조작성 w = {manipulability(0.6, 1.3):.4f}")
    print(f"  특이점  θ2=0.0   조작성 w = {manipulability(0.6, 0.0):.4f} (≈0 → 자유도 손실)")
    print(f"  특이점  θ2=π     조작성 w = {manipulability(0.6, np.pi):.4f} (≈0 → 자유도 손실)")
    plot_all()
