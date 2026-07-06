"""
A 단계 실험: 메카넘 드라이브트레인의 야코비안 분석
  ① 유사역행렬(pseudo-inverse) 순기구학  = 오도메트리의 수학적 뿌리
  ② Jᵀ 영공간(null space) 기반 바퀴 슬립 검출
  ③ 조작성 타원체(manipulability ellipsoid)

핵심 아이디어 (세특 탐구 요지):
  메카넘은 바퀴 4개 / 자유도 3개 → 야코비안 J 가 4x3 (과결정, over-determined).
  이 때문에 정방행렬 로봇팔과 달리 특이점 대신 "유사역행렬·영공간"이라는
  다른 야코비안 이론이 등장한다. 이걸 실제 로봇으로 검증하는 게 이 실험이다.

단위:
  vx, vy: m/s (로봇 로컬, +forward / +left)
  wz    : rad/s (+CCW)
  wheel angular vel: rad/s
"""

import sys
import numpy as np
import matplotlib.pyplot as plt

# Windows 콘솔(cp949)에서도 유니코드 수식기호(⁺, ᵀ 등)를 출력할 수 있게 UTF-8 강제
try:
    sys.stdout.reconfigure(encoding="utf-8")
except AttributeError:
    pass

# matplotlib 그래프에 한글이 깨지지 않도록 Windows 기본 한글 폰트 지정
plt.rcParams["font.family"] = "Malgun Gothic"
plt.rcParams["axes.unicode_minus"] = False  # 마이너스 기호 깨짐 방지

# =============================================================================
# 하드웨어 파라미터 (실측 필요 — placeholder, CLAUDE.md 규칙 1)
# =============================================================================
# TODO: 실측 필요 — 구동 바퀴 반지름 (goBILDA 메카넘 96mm 기준 이론값 예시)
WHEEL_RADIUS = 0.048    # m
# TODO: 실측 필요 — 로봇 회전 레버암 L = (트랙폭/2 + 휠베이스/2)
#   트랙폭(좌우 바퀴 간격), 휠베이스(앞뒤 바퀴 간격)를 실제로 재서 넣을 것
HALF_TRACK = 0.18       # m (좌우 간격의 절반, 임시값)
HALF_BASE = 0.16        # m (앞뒤 간격의 절반, 임시값)
L = HALF_TRACK + HALF_BASE   # 회전 레버암 (m)

r = WHEEL_RADIUS


# =============================================================================
# 야코비안 정의
#   [r*ω_lf]     [1  -1  -L] [vx]
#   [r*ω_rf]  =  [1   1   L] [vy]
#   [r*ω_lb]     [1   1  -L] [wz]
#   [r*ω_rb]     [1  -1   L]
#   좌변 = 바퀴 접지점 선속도(m/s), 우변 = 섀시 속도.  이 4x3 행렬이 J.
# =============================================================================
J = np.array([
    [1, -1, -L],
    [1,  1,  L],
    [1,  1, -L],
    [1, -1,  L],
], dtype=float)


def chassis_to_wheels(vx, vy, wz):
    """역기구학: 섀시 속도 -> 바퀴 각속도 (rad/s). J 를 곱하고 반지름으로 나눔."""
    wheel_linear = J @ np.array([vx, vy, wz])   # 바퀴 선속도 (m/s)
    return wheel_linear / r                      # 바퀴 각속도 (rad/s)


def wheels_to_chassis(wheel_omega):
    """
    ① 순기구학: 바퀴 각속도 4개 -> 섀시 속도 3개, 유사역행렬 J⁺ 사용.
    과결정(4>3)이라 정확한 역행렬이 없어 최소제곱해를 준다: J⁺ = (JᵀJ)⁻¹Jᵀ.
    (이게 오도메트리가 바퀴 4개 정보를 '최소제곱'으로 융합하는 원리다.)
    """
    wheel_linear = wheel_omega * r
    J_pinv = np.linalg.pinv(J)
    return J_pinv @ wheel_linear   # [vx, vy, wz]


# ② Jᵀ의 영공간: 섀시를 전혀 움직이지 않는 바퀴속도 조합 = 순수 내부 슬립 방향
#    (4-3=1 차원). 측정된 바퀴속도의 이 방향 성분이 크면 = 슬립/스크럽 발생 신호.
def null_space_of_JT():
    # Jᵀ (3x4)의 영공간 = J의 좌영공간. SVD로 구함.
    u, s, vt = np.linalg.svd(J)          # J = U S Vᵀ, U는 4x4
    # 특이값이 0에 대응하는 U의 열이 좌영공간 (rank=3 이므로 마지막 열)
    null_dir = u[:, 3]
    return null_dir / np.linalg.norm(null_dir)


def slip_metric(wheel_omega, null_dir):
    """측정된 바퀴 각속도를 영공간 방향에 투영한 크기 = 슬립 지표 (rad/s)."""
    return float(np.dot(wheel_omega, null_dir))


# =============================================================================
# 검증 (규칙 4: assert)
# =============================================================================
def run_asserts():
    # (1) 역기구학->순기구학 왕복: 슬립 없으면 원래 섀시 속도 복원돼야 함
    vx, vy, wz = 0.7, -0.3, 1.2
    wheels = chassis_to_wheels(vx, vy, wz)
    recovered = wheels_to_chassis(wheels)
    assert np.allclose(recovered, [vx, vy, wz]), f"왕복 복원 실패: {recovered}"

    # (2) J⁺J = I (3x3): 유사역행렬이 좌역행렬임을 확인
    assert np.allclose(np.linalg.pinv(J) @ J, np.eye(3)), "J⁺J != I"

    # (3) 영공간 방향은 Jᵀ n = 0 을 만족해야 함
    n = null_space_of_JT()
    assert np.allclose(J.T @ n, np.zeros(3), atol=1e-9), f"Jᵀn != 0: {J.T @ n}"

    # (4) 영공간 방향은 이론값 [1,1,-1,-1]/2 와 평행해야 함 (앞바퀴 vs 뒷바퀴)
    theory = np.array([1, 1, -1, -1]) / 2.0
    assert np.allclose(np.abs(n), np.abs(theory)), f"영공간 이론 불일치: {n}"

    print("[검증 통과] 왕복복원 / J⁺J=I / Jᵀn=0 / 영공간=[1,1,-1,-1]")
    print(f"           영공간 방향 (슬립 모드): {np.round(n, 3)}  (앞바퀴 vs 뒷바퀴 스크럽)")


# =============================================================================
# ② 슬립 검출 데모: 정상 주행 vs 앞바퀴만 슬립난 경우
# =============================================================================
def slip_demo():
    null_dir = null_space_of_JT()

    # 정상: 순수 전진 명령대로 바퀴가 굴러감
    clean = chassis_to_wheels(0.6, 0.0, 0.0)
    # 슬립: 앞바퀴(lf,rf)가 명령보다 20% 헛돎 (접지 불량 모사)
    slipped = clean.copy()
    slipped[0] *= 1.2
    slipped[1] *= 1.2

    print("\n[② 슬립 검출]")
    print(f"  정상 주행 슬립지표 : {slip_metric(clean, null_dir): .4f} rad/s (≈0 정상)")
    print(f"  앞바퀴 슬립 슬립지표: {slip_metric(slipped, null_dir): .4f} rad/s (0에서 벗어남 → 슬립 감지)")
    return clean, slipped, null_dir


# =============================================================================
# ③ 조작성 타원체
# =============================================================================
def manipulability():
    # 단위 바퀴속도(||ω||=1)로 낼 수 있는 섀시속도 집합의 형상 = J⁺ 의 특이값
    u, s, vt = np.linalg.svd(J)
    # J의 특이값 s -> J⁺의 특이값은 1/s. 섀시속도 이득 = r/s (반지름 반영)
    sigma = s
    print("\n[③ 조작성 타원체]")
    print(f"  J 특이값 σ = {np.round(sigma, 3)}  (이론값 [2, 2, 2L]={np.round([2,2,2*L],3)})")
    print(f"  → 병진(vx,vy)은 등방(같은 σ=2), 회전은 σ=2L={2*L:.3f} 로 L에 비례")
    print(f"  → 로봇이 커질수록(L↑) 같은 바퀴속도로 낼 수 있는 각속도 감소")
    return sigma


def plot_all(clean, slipped, null_dir, sigma):
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    # (1) 슬립 검출: 바퀴속도 벡터를 영공간 방향과 비교
    ax = axes[0]
    idx = np.arange(4)
    width = 0.35
    ax.bar(idx - width/2, clean, width, label='정상(clean)', color='steelblue')
    ax.bar(idx + width/2, slipped, width, label='앞바퀴 슬립', color='indianred')
    ax.set_xticks(idx); ax.set_xticklabels(['lf', 'rf', 'lb', 'rb'])
    ax.set_ylabel('wheel ω (rad/s)')
    ax.set_title(f'② Slip Detection\nnull-dir={np.round(null_dir,2)}\n'
                 f'metric: clean={np.dot(clean,null_dir):.3f}, slip={np.dot(slipped,null_dir):.3f}')
    ax.legend(); ax.grid(True, alpha=0.3)

    # (2) 병진 조작성 타원 (vx-vy 평면): 이상적으론 등방(원)
    ax = axes[1]
    theta = np.linspace(0, 2*np.pi, 200)
    # 단위 바퀴 노름으로 낼 수 있는 병진속도 반경 = r/σ_translation = r/2
    rad = r / 2.0
    ax.plot(rad*np.cos(theta), rad*np.sin(theta), 'g-', linewidth=2)
    ax.arrow(0, 0, rad, 0, head_width=0.005, color='b')
    ax.arrow(0, 0, 0, rad, head_width=0.005, color='b')
    ax.text(rad*0.6, 0.004, 'vx', color='b'); ax.text(0.004, rad*0.6, 'vy', color='b')
    ax.set_xlabel('vx (m/s)'); ax.set_ylabel('vy (m/s)')
    ax.set_title('③ Translation Manipulability\n(이론상 등방 → 원)\n'
                 '실제는 롤러 마찰로 찌그러짐(실측 과제)')
    ax.axis('equal'); ax.grid(True, alpha=0.3)

    # (3) 특이값 막대
    ax = axes[2]
    ax.bar(['σ_vx', 'σ_vy', 'σ_wz'], sigma, color=['seagreen', 'seagreen', 'darkorange'])
    ax.axhline(2, color='gray', ls=':', alpha=0.6)
    ax.set_ylabel('singular value')
    ax.set_title(f'③ Singular Values of J\n회전 σ=2L={2*L:.2f} (L={L:.2f}m)')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = "sim/jacobian_experiment_result.png"
    plt.savefig(out, dpi=110)
    print(f"\n[저장] {out}")


if __name__ == "__main__":
    print(f"파라미터(전부 실측 전 placeholder): r={r}m, L={L}m")
    run_asserts()
    clean, slipped, null_dir = slip_demo()
    sigma = manipulability()
    plot_all(clean, slipped, null_dir, sigma)
