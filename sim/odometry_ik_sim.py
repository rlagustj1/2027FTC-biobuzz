"""
1단계 시뮬레이션: 메카넘 역기구학 + 2륜 데드휠/IMU 오도메트리 + 경로추종 P제어

목적 (CLAUDE.md 규칙 4):
  웹 대시보드 / Java 이식 전에, 좌표를 넣으면 로봇이 그 (x, y, heading)까지
  이론적으로 잘 가는지, 오도메트리 추정이 실제 위치를 잘 따라가는지 그래프로 검증한다.

좌표계 정의:
  - 필드 좌표 (X, Y): 미터(m). X = 전방(+forward), Y = 좌측(+left). 오른손 좌표계.
  - heading θ: 라디안(rad). +반시계(CCW). 로봇이 +X를 바라보면 θ=0.
  - 모든 각속도: rad/s, 선속도: m/s, 시간: s.

주의 (CLAUDE.md 규칙 1):
  아래 하드웨어 파라미터는 전부 실측 전 placeholder다. 실제 로봇 튜닝 전까지 최종값 아님.
"""

import numpy as np
import matplotlib.pyplot as plt

# =============================================================================
# 하드웨어 파라미터 (전부 실측 필요 — placeholder)
# =============================================================================
# TODO: 실측 필요 — 데드휠 오도메트리 휠 반지름 (goBILDA 오도팟 기준 이론값 예시)
DEAD_WHEEL_RADIUS = 0.024      # m (예: 지름 48mm 오도휠 → 반지름 0.024m)
# TODO: 실측 필요 — 오도메트리 엔코더 1회전당 틱수 (REV through-bore 등)
ODO_TICKS_PER_REV = 8192       # 틱/회전
# TODO: 실측 필요 — 로봇 중심 기준 데드휠 장착 오프셋
#   PARALLEL_OFFSET: 전진(parallel) 휠이 중심에서 Y방향으로 떨어진 거리 (m)
#   PERP_OFFSET:     스트레이프(perpendicular) 휠이 중심에서 X방향으로 떨어진 거리 (m)
PARALLEL_OFFSET = 0.08         # m (임시값)
PERP_OFFSET = -0.10            # m (임시값, 부호는 장착 방향에 따라 실측으로 결정)

# 파생값: 오도휠 1틱당 이동 거리 (m/틱)
ODO_M_PER_TICK = (2 * np.pi * DEAD_WHEEL_RADIUS) / ODO_TICKS_PER_REV

# 섀시 물리 한계 (경로추종 P제어 출력 clamp용) — 실측 필요
VMAX = 1.0     # m/s   (임시값, 실제 로봇 최고 병진속도 측정 필요)
WMAX = 3.0     # rad/s (임시값, 실제 로봇 최고 회전속도 측정 필요)

# 제어 루프 주기 (FTC 제어 루프 ~50Hz 가정)
DT = 0.02      # s


# =============================================================================
# 1) 메카넘 역기구학: 섀시 속도 (vx, vy, w) -> 바퀴 4속도
# =============================================================================
def mecanum_inverse_kinematics(vx, vy, w):
    """
    로봇 로컬 프레임 기준 섀시 속도를 바퀴 4개의 (정규화된) 속도로 변환.

    :param vx: 로봇 전방 속도 (m/s, +forward)
    :param vy: 로봇 좌측 속도 (m/s, +left)  ← strafe
    :param w:  회전 각속도 (rad/s, +CCW)
    :return: (lf, rf, lb, rb) 바퀴 속도 배열 (모터 출력에 비례하는 상대값)

    표준 X-드라이브 메카넘 식. 실제 모터 출력(-1~1)으로 쓰려면 최댓값으로 정규화 필요.
    """
    lf = vx - vy - w
    rf = vx + vy + w
    lb = vx + vy - w
    rb = vx - vy + w
    return np.array([lf, rf, lb, rb])


# =============================================================================
# 2) 시뮬레이션용 "실제 로봇" 모델 (ground truth)
#    실제 로봇은 우리가 명령한 섀시 속도대로 (약간의 오차와 함께) 움직인다고 가정.
# =============================================================================
def step_ground_truth(state, vx_cmd, vy_cmd, w_cmd, dt, slip=0.0, noise=0.0):
    """
    명령 섀시 속도를 받아 실제 필드 좌표를 적분(오일러)으로 갱신.

    :param state: (X, Y, theta) 필드 좌표 (m, m, rad)
    :param vx_cmd, vy_cmd: 로봇 로컬 프레임 명령 속도 (m/s)
    :param w_cmd: 명령 각속도 (rad/s)
    :param dt: 적분 스텝 (s)
    :param slip: 바퀴 슬립 계수 (0=완벽, 0.05=5% 미끄러짐) — 오차 원인 모사
    :param noise: 속도에 섞이는 가우시안 노이즈 표준편차
    :return: 갱신된 (X, Y, theta)
    """
    X, Y, th = state
    # 슬립/노이즈로 실제 이동은 명령과 살짝 다르다
    eff = (1.0 - slip)
    vx = vx_cmd * eff + np.random.randn() * noise
    vy = vy_cmd * eff + np.random.randn() * noise
    w = w_cmd * eff

    # 로컬 -> 필드 프레임 회전 변환 후 적분
    X += (vx * np.cos(th) - vy * np.sin(th)) * dt
    Y += (vx * np.sin(th) + vy * np.cos(th)) * dt
    th += w * dt
    return np.array([X, Y, th])


# =============================================================================
# 3) 2륜 데드휠 + IMU 오도메트리
#    - parallel 휠: 로봇 전진(local x) 측정
#    - perpendicular 휠: 로봇 스트레이프(local y) 측정 (단, 회전 성분 보정 필요)
#    - heading: IMU에서 직접 읽음 (여기선 ground truth theta + 노이즈로 모사)
# =============================================================================
class TwoWheelOdometry:
    def __init__(self):
        self.X = 0.0
        self.Y = 0.0
        self.theta = 0.0
        self.prev_theta = 0.0

    def update(self, d_parallel_ticks, d_perp_ticks, imu_heading):
        """
        엔코더 틱 변화량 + IMU heading으로 필드 좌표 추정 갱신.

        :param d_parallel_ticks: parallel 데드휠 틱 변화량 (틱)
        :param d_perp_ticks: perpendicular 데드휠 틱 변화량 (틱)
        :param imu_heading: IMU가 보고한 절대 heading (rad)

        원리: perpendicular 휠은 순수 스트레이프뿐 아니라 로봇 회전(Δθ)에 의해서도
        굴러간다. 회전 성분(PERP_OFFSET * Δθ)을 빼줘야 순수 y 이동이 나온다.
        마찬가지로 parallel 휠도 PARALLEL_OFFSET * Δθ 보정.
        """
        dth = imu_heading - self.prev_theta
        # 정규화 (-pi ~ pi) — heading 랩어라운드 방지
        dth = (dth + np.pi) % (2 * np.pi) - np.pi

        d_par = d_parallel_ticks * ODO_M_PER_TICK
        d_perp = d_perp_ticks * ODO_M_PER_TICK

        # 회전에 의한 데드휠 회전분 보정 -> 순수 로컬 병진 성분
        local_dx = d_par - PARALLEL_OFFSET * dth
        local_dy = d_perp - PERP_OFFSET * dth

        # 이동 구간 중앙 heading으로 로컬->필드 변환 (1차 근사)
        th_mid = self.prev_theta + dth / 2.0
        self.X += local_dx * np.cos(th_mid) - local_dy * np.sin(th_mid)
        self.Y += local_dx * np.sin(th_mid) + local_dy * np.cos(th_mid)
        self.theta = imu_heading
        self.prev_theta = imu_heading

    def pose(self):
        return np.array([self.X, self.Y, self.theta])


# =============================================================================
# 4) 경로추종 P제어: 목표 (x, y, heading)까지 섀시 속도 명령 생성
# =============================================================================
# TODO: 실측 튜닝 필요 — 위치/방향 P게인. 시뮬레이션 검증용 임시값.
KP_POS = 2.0     # 위치 오차 -> 병진속도 게인
KP_HEADING = 3.0 # 방향 오차 -> 각속도 게인

def path_follow_control(est_pose, target):
    """
    현재 추정 pose와 목표 pose로부터 로봇 로컬 프레임 섀시 속도 명령 계산.

    :param est_pose: 오도메트리 추정 (X, Y, theta)
    :param target: 목표 (X, Y, theta)
    :return: (vx, vy, w) 로봇 로컬 프레임 명령 (m/s, m/s, rad/s)

    원리: 필드 좌표 오차를 로봇 로컬 프레임으로 회전변환한 뒤 P제어.
    heading 오차는 최단 회전방향으로 정규화.
    """
    X, Y, th = est_pose
    tx, ty, tth = target

    err_field = np.array([tx - X, ty - Y])
    # 필드 오차 -> 로봇 로컬 프레임 (역회전)
    c, s = np.cos(-th), np.sin(-th)
    ex_local = err_field[0] * c - err_field[1] * s
    ey_local = err_field[0] * s + err_field[1] * c

    vx = np.clip(KP_POS * ex_local, -VMAX, VMAX)
    vy = np.clip(KP_POS * ey_local, -VMAX, VMAX)

    heading_err = (tth - th + np.pi) % (2 * np.pi) - np.pi
    w = np.clip(KP_HEADING * heading_err, -WMAX, WMAX)
    return vx, vy, w


# =============================================================================
# 시뮬레이션 루프
# =============================================================================
def simulate(target, sim_time=6.0, slip=0.03, vel_noise=0.01, imu_noise=0.005, seed=0):
    np.random.seed(seed)

    gt = np.array([0.0, 0.0, 0.0])       # ground truth pose
    odo = TwoWheelOdometry()

    log = {"t": [], "gt": [], "odo": [], "cmd": []}

    steps = int(sim_time / DT)
    for i in range(steps):
        # --- 제어: 오도메트리 추정 기반으로 명령 생성 ---
        vx, vy, w = path_follow_control(odo.pose(), target)

        # --- 실제 로봇(ground truth) 갱신 ---
        prev_gt = gt.copy()
        gt = step_ground_truth(gt, vx, vy, w, DT, slip=slip, noise=vel_noise)

        # --- 센서 시뮬레이션: 실제 이동량을 데드휠 틱으로 역산 ---
        # 실제 로컬 프레임 이동량 계산 (ground truth 기준)
        dX, dY = gt[0] - prev_gt[0], gt[1] - prev_gt[1]
        th_prev = prev_gt[2]
        c, s = np.cos(-th_prev), np.sin(-th_prev)
        local_dx = dX * c - dY * s
        local_dy = dX * s + dY * c
        dth_real = gt[2] - prev_gt[2]

        # 데드휠이 실제로 굴러간 거리 = 순수 병진 + 회전에 의한 성분
        par_dist = local_dx + PARALLEL_OFFSET * dth_real
        perp_dist = local_dy + PERP_OFFSET * dth_real
        d_par_ticks = par_dist / ODO_M_PER_TICK
        d_perp_ticks = perp_dist / ODO_M_PER_TICK

        # IMU heading (노이즈 포함)
        imu_heading = gt[2] + np.random.randn() * imu_noise

        # --- 오도메트리 갱신 ---
        odo.update(d_par_ticks, d_perp_ticks, imu_heading)

        log["t"].append(i * DT)
        log["gt"].append(gt.copy())
        log["odo"].append(odo.pose())
        log["cmd"].append([vx, vy, w])

    for k in log:
        log[k] = np.array(log[k])
    return log


def plot_results(log, target):
    gt = log["gt"]
    odo = log["odo"]
    t = log["t"]

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))

    # (1) 필드 궤적: ground truth vs 오도메트리 추정
    ax = axes[0, 0]
    ax.plot(gt[:, 0], gt[:, 1], 'b-', label='Ground Truth', linewidth=2)
    ax.plot(odo[:, 0], odo[:, 1], 'r--', label='Odometry Est.', linewidth=1.5)
    ax.plot(0, 0, 'ks', markersize=8, label='Start')
    ax.plot(target[0], target[1], 'g*', markersize=18, label='Target')
    ax.set_xlabel('Field X (m, forward)')
    ax.set_ylabel('Field Y (m, left)')
    ax.set_title('Field Trajectory')
    ax.legend(); ax.grid(True); ax.axis('equal')

    # (2) X, Y 시간 그래프
    ax = axes[0, 1]
    ax.plot(t, gt[:, 0], 'b-', label='X gt')
    ax.plot(t, odo[:, 0], 'b--', label='X odo')
    ax.plot(t, gt[:, 1], 'g-', label='Y gt')
    ax.plot(t, odo[:, 1], 'g--', label='Y odo')
    ax.axhline(target[0], color='b', ls=':', alpha=0.5)
    ax.axhline(target[1], color='g', ls=':', alpha=0.5)
    ax.set_xlabel('t (s)'); ax.set_ylabel('position (m)')
    ax.set_title('Position vs Time'); ax.legend(); ax.grid(True)

    # (3) heading 시간 그래프
    ax = axes[1, 0]
    ax.plot(t, np.degrees(gt[:, 2]), 'b-', label='heading gt')
    ax.plot(t, np.degrees(odo[:, 2]), 'r--', label='heading odo')
    ax.axhline(np.degrees(target[2]), color='k', ls=':', alpha=0.5, label='target')
    ax.set_xlabel('t (s)'); ax.set_ylabel('heading (deg)')
    ax.set_title('Heading vs Time'); ax.legend(); ax.grid(True)

    # (4) 오도메트리 오차 (추정 - 실제)
    ax = axes[1, 1]
    err = np.linalg.norm(gt[:, :2] - odo[:, :2], axis=1)
    ax.plot(t, err * 1000, 'm-')
    ax.set_xlabel('t (s)'); ax.set_ylabel('position error (mm)')
    ax.set_title('Odometry Drift (|gt - odo|)'); ax.grid(True)

    plt.tight_layout()
    out = "sim/odometry_ik_result.png"
    plt.savefig(out, dpi=110)
    print(f"[저장] {out}")


# =============================================================================
# 수치 검증 (CLAUDE.md 규칙 4: assert로 이론값 확인)
# =============================================================================
def run_asserts():
    # 역기구학 검증: 순수 전진이면 4바퀴 모두 같은 부호/크기
    lf, rf, lb, rb = mecanum_inverse_kinematics(1.0, 0.0, 0.0)
    assert np.allclose([lf, rf, lb, rb], [1, 1, 1, 1]), "전진 IK 실패"

    # 순수 스트레이프(+left)면 lf,rb 음 / rf,lb 양
    w = mecanum_inverse_kinematics(0.0, 1.0, 0.0)
    assert np.allclose(w, [-1, 1, 1, -1]), f"스트레이프 IK 실패: {w}"

    # 순수 회전(+CCW)이면 좌측(lf,lb) 음 / 우측(rf,rb) 양
    w = mecanum_inverse_kinematics(0.0, 0.0, 1.0)
    assert np.allclose(w, [-1, 1, -1, 1]), f"회전 IK 실패: {w}"

    print("[검증 통과] 메카넘 역기구학 3방향(전진/스트레이프/회전) 부호 정상")


if __name__ == "__main__":
    run_asserts()

    # 목표: X=1.0m 전방, Y=0.5m 좌측, heading=90도(CCW)
    target = np.array([1.0, 0.5, np.radians(90)])
    log = simulate(target)

    final_gt = log["gt"][-1]
    final_odo = log["odo"][-1]
    print(f"\n목표 pose      : X={target[0]:.3f} Y={target[1]:.3f} th={np.degrees(target[2]):.1f}deg")
    print(f"실제 도달 pose : X={final_gt[0]:.3f} Y={final_gt[1]:.3f} th={np.degrees(final_gt[2]):.1f}deg")
    print(f"오도 추정 pose : X={final_odo[0]:.3f} Y={final_odo[1]:.3f} th={np.degrees(final_odo[2]):.1f}deg")
    print(f"오도 최종 오차 : {np.linalg.norm(final_gt[:2]-final_odo[:2])*1000:.1f} mm")

    plot_results(log, target)
