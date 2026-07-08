"""
보고서 Ⅲ.3 — 목표 궤적 vs 실측 궤적 비교 그래프 생성기.

입력: pc/dashboard.py --log 로 저장한 CSV
  컬럼: t, target_x, target_y, target_h_deg, x, y, h_deg
출력: sim/trajectory_compare_result.png
  ① XY 궤적 겹침 (목표점 vs 실측 경로)
  ② heading 시간응답 (목표 vs 실측)
  ③ 위치 오차 시간응답 (목표까지 거리)

사용:
  python sim/trajectory_compare.py runs/run1.csv
  python sim/trajectory_compare.py            # CSV 없으면 데모 데이터로 형태 미리보기

※ 로봇 불필요. 데이터만 꽂으면 보고서 Ⅲ.3 표/그래프가 완성된다.
"""

import sys
import os
import csv
import numpy as np
import matplotlib.pyplot as plt

try:
    sys.stdout.reconfigure(encoding="utf-8")
except AttributeError:
    pass

plt.rcParams["font.family"] = "Malgun Gothic"
plt.rcParams["axes.unicode_minus"] = False


def load_csv(path):
    """CSV -> dict of np arrays. 빈 목표칸은 nan 처리."""
    cols = {k: [] for k in ["t", "target_x", "target_y", "target_h_deg", "x", "y", "h_deg"]}
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            for k in cols:
                v = row.get(k, "")
                cols[k].append(float(v) if v not in ("", None) else np.nan)
    return {k: np.array(v) for k, v in cols.items()}


def make_demo():
    """
    데모 데이터 (실측 전 형태 미리보기용, 명확히 SIM 표기).
    목표 (0, -0.4, 0°) 로 가는 1차 지연 응답 + 노이즈로 '그럴듯한' 궤적 생성.
    """
    t = np.linspace(0, 3.0, 150)
    tx, ty, th_t = 0.0, -0.4, 0.0
    # 1차 지연으로 목표에 수렴 (시정수 0.6s) + 소량 노이즈
    x = tx * (1 - np.exp(-t / 0.6)) + 0.015 * np.sin(6 * t)
    y = ty * (1 - np.exp(-t / 0.6)) + 0.02 * np.sin(5 * t + 1)
    h = th_t + 4.0 * np.exp(-t / 0.5) * np.sin(8 * t)   # 초반 흔들리다 0으로
    return {
        "t": t,
        "target_x": np.full_like(t, tx), "target_y": np.full_like(t, ty),
        "target_h_deg": np.full_like(t, th_t),
        "x": x, "y": y, "h_deg": h,
    }, True


def plot(d, is_demo):
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    tag = "  [DEMO — 실측 데이터로 교체 필요]" if is_demo else ""

    # ① XY 궤적 겹침
    ax = axes[0]
    ax.plot(d["x"], d["y"], "-", color="tab:blue", lw=1.8, label="실측 경로(오도)")
    ax.plot(d["x"][0], d["y"][0], "go", ms=10, label="시작")
    ax.plot(d["x"][-1], d["y"][-1], "bo", ms=8, label="종료")
    # 목표점 (마지막 유효 목표)
    tvx, tvy = d["target_x"], d["target_y"]
    valid = ~np.isnan(tvx)
    if valid.any():
        gx, gy = tvx[valid][-1], tvy[valid][-1]
        ax.plot(gx, gy, "r*", ms=18, label="목표")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title("① 목표 vs 실측 궤적" + tag)
    ax.axis("equal"); ax.grid(True, alpha=0.3); ax.legend(fontsize=8)

    # ② heading 시간응답
    ax = axes[1]
    ax.plot(d["t"], d["h_deg"], "-", color="tab:blue", lw=1.6, label="실측 heading")
    ax.plot(d["t"], d["target_h_deg"], "--", color="tab:red", lw=1.4, label="목표 heading")
    ax.set_xlabel("t (s)"); ax.set_ylabel("heading (°)")
    ax.set_title("② heading 시간응답\n(부호 수정 후 발산 없이 수렴 확인)")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8)

    # ③ 위치 오차 시간응답
    ax = axes[2]
    err = np.hypot(d["target_x"] - d["x"], d["target_y"] - d["y"])
    ax.plot(d["t"], err, "-", color="tab:purple", lw=1.8)
    ax.axhline(0.03, color="gray", ls=":", label="허용오차 3cm")
    ax.set_xlabel("t (s)"); ax.set_ylabel("목표까지 거리 (m)")
    ax.set_title("③ 위치 오차 수렴")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8)

    plt.tight_layout()
    out = "sim/trajectory_compare_result.png"
    plt.savefig(out, dpi=110)
    print(f"[저장] {out}")

    # 요약 통계 (보고서 표에 그대로 사용)
    print("\n[요약 — 보고서 Ⅲ.3 표에 기입]")
    print(f"  최종 위치오차 : {err[-1]:.3f} m")
    print(f"  최종 heading  : {d['h_deg'][-1]:.1f}° (목표 {d['target_h_deg'][-1]:.1f}°)")
    print(f"  정착시간(오차<3cm 진입): "
          f"{_settle_time(d['t'], err):.2f} s" if (err < 0.03).any() else "  정착시간: 미도달")


def _settle_time(t, err, tol=0.03):
    idx = np.argmax(err < tol)   # 처음으로 tol 밑으로 내려간 인덱스
    return t[idx]


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else None
    if path and os.path.exists(path):
        print(f"[입력] {path}")
        d, is_demo = load_csv(path), False
    else:
        if path:
            print(f"[경고] '{path}' 없음 → 데모 데이터로 형태만 미리보기")
        else:
            print("[안내] CSV 미지정 → 데모 데이터. 실제: python sim/trajectory_compare.py runs/run1.csv")
        d, is_demo = make_demo()
    plot(d, is_demo)
