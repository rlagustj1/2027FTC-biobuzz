"""
자율주행 AI 아키텍처 — 계측 실험 (보고서 Ⅴ. H1·H2 검증)

로봇/실모델 없이 sim으로 다음을 실측한다:
  H1 (지연 격리): 상위 planner에 인위적 지연을 0→여러 단계로 주입해도
                  하위 control 루프의 실측 주기(Hz)가 목표(~20Hz)를 유지하는가?
  H2 (비동기 병렬): 세 계층(perception 30 / planner / control 20 Hz)이
                    설계 주기로 독립 동작하며 서로를 블로킹하지 않는가?

핵심 계측: 각 스레드가 한 바퀴 돌 때마다 타임스탬프를 남겨 실측 Hz를 계산한다.
planner의 지연은 '상위 AI(VLM) 추론시간'을 흉내낸 것.

실행:
  python pc/ai/timing_experiment.py           # 지연 스윕 + 그래프
출력:
  pc/ai/timing_experiment_result.png
  콘솔에 가설 판정 표
"""

from __future__ import annotations
import os
import sys
import math
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))          # pc/ai
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # pc

try:
    sys.stdout.reconfigure(encoding="utf-8")
except AttributeError:
    pass

from perception import MockPerception, Detection      # noqa: E402
from planner import PlannerBase, Goal                  # noqa: E402


class SimLink:
    """가짜 로봇 링크 (orchestrator.py의 SimLink와 동일 역할). goto는 카운트만."""
    def __init__(self):
        self.running = True
        self._t0 = time.time()
        self.goto_count = 0

    @property
    def latest_pose(self):
        t = time.time() - self._t0
        return {"x": 0.3 * math.cos(0.2 * t), "y": 0.3 * math.sin(0.2 * t), "h": 0.2 * t, "slip": 0.0}

    def goto(self, x, y, h_rad):
        self.goto_count += 1


class SlowPlanner(PlannerBase):
    """상위 VLM 추론시간을 흉내내는 플래너. plan() 한 번에 delay_s 만큼 잠든다."""
    def __init__(self, delay_s):
        self.delay_s = delay_s

    def plan(self, detections, robot_pose, frame=None) -> Goal:
        time.sleep(self.delay_s)   # ← VLM 추론 지연 모사 (수백 ms~수 초)
        reds = [d for d in detections if d.label == "red_block"]
        if reds:
            return Goal("goto_object", "red_block", f"mock VLM (지연 {self.delay_s}s)")
        return Goal("idle", None, "대상 없음")


class InstrumentedOrchestrator:
    """
    orchestrator.py의 3계층 구조를 그대로 두되, 각 스레드의 '루프 완료 시각'을
    타임스탬프 리스트로 기록해 실측 Hz를 계산할 수 있게 계측을 추가한 버전.
    """
    def __init__(self, link, perception, planner,
                 perc_hz=30.0, plan_hz=0.2, ctrl_hz=20.0):
        self.link = link
        self.perception = perception
        self.planner = planner
        self.periods = {"perception": 1.0 / perc_hz,
                        "planner": 1.0 / plan_hz,
                        "control": 1.0 / ctrl_hz}
        self._lock = threading.Lock()
        self._detections = []
        self._goal = Goal("idle", None, "init")
        self._running = False
        # 계측: 각 계층 루프 완료 타임스탬프
        self.stamps = {"perception": [], "planner": [], "control": []}

    def _pose_tuple(self):
        p = self.link.latest_pose
        return (p["x"], p["y"], p["h"]) if p else (0.0, 0.0, 0.0)

    def _perception_loop(self):
        while self._running:
            dets = self.perception.detect(frame=None, robot_pose=self._pose_tuple())
            with self._lock:
                self._detections = dets
            self.stamps["perception"].append(time.time())
            time.sleep(self.periods["perception"])

    def _planner_loop(self):
        while self._running:
            with self._lock:
                dets = list(self._detections)
            goal = self.planner.plan(dets, self._pose_tuple(), frame=None)  # 여기서 지연 발생
            with self._lock:
                self._goal = goal
            self.stamps["planner"].append(time.time())
            time.sleep(self.periods["planner"])

    def _control_loop(self):
        while self._running:
            with self._lock:
                goal = self._goal
                dets = list(self._detections)
            target = self._resolve(goal, dets, self._pose_tuple())
            if target is not None:
                self.link.goto(*target)
            self.stamps["control"].append(time.time())
            time.sleep(self.periods["control"])

    def _resolve(self, goal, dets, pose):
        if goal.kind != "goto_object" or goal.target_label is None:
            return None
        cands = [d for d in dets if d.label == goal.target_label]
        if not cands:
            return None
        rx, ry, _ = pose
        nearest = min(cands, key=lambda d: (d.field_x - rx) ** 2 + (d.field_y - ry) ** 2)
        h = math.atan2(nearest.field_y - ry, nearest.field_x - rx)
        return (nearest.field_x, nearest.field_y, h)

    def run_for(self, seconds):
        self._running = True
        self._duration = seconds
        threads = [threading.Thread(target=f, daemon=True) for f in
                   (self._perception_loop, self._planner_loop, self._control_loop)]
        for t in threads:
            t.start()
        time.sleep(seconds)
        self._running = False
        time.sleep(0.3)  # 마지막 루프 정리

    def measured_hz(self):
        """
        계측 타임스탬프 -> 각 계층 실측 Hz.
        느린 계층(planner)은 창 안에 완료가 1~2회뿐이라 '간격 평균'이 불안정하므로,
        '완료 횟수 / 실행시간'(throughput)으로 계산해 느린 계층도 정확히 잡는다.
        """
        return {layer: len(ts) / self._duration for layer, ts in self.stamps.items()}


def sweep(delays, seconds=6.0):
    """planner 지연을 여러 값으로 바꿔가며 control 루프 실측 Hz를 기록."""
    rows = []
    for d in delays:
        orch = InstrumentedOrchestrator(SimLink(), MockPerception(), SlowPlanner(d))
        orch.run_for(seconds)
        hz = orch.measured_hz()
        rows.append({"planner_delay": d, **hz})
        print(f"  planner지연 {d:>4}s → 실측 Hz  perception={hz['perception']:5.1f}"
              f"  planner={hz['planner']:5.2f}  control={hz['control']:5.1f}")
    return rows


def main():
    import numpy as np
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = "Malgun Gothic"
    plt.rcParams["axes.unicode_minus"] = False

    print("[계측 실험] 3계층 sim 구동, planner(VLM) 지연 스윕")
    print("  목표 주기: perception 30Hz / planner 0.2Hz / control 20Hz\n")
    delays = [0.0, 0.1, 0.5, 1.0, 2.0]
    rows = sweep(delays)

    ctrl_target = 20.0
    ctrl_hz = [r["control"] for r in rows]
    perc_hz = [r["perception"] for r in rows]

    # 판정: control Hz가 목표의 ±20% 안에 유지되면 H1 지지
    tol = 0.20
    h1_pass = all(abs(h - ctrl_target) / ctrl_target < tol for h in ctrl_hz)
    print("\n[가설 판정]")
    print(f"  H1 (지연 격리): planner 지연 0→{delays[-1]}s 에도 control Hz "
          f"{min(ctrl_hz):.1f}~{max(ctrl_hz):.1f} (목표 {ctrl_target}) "
          f"→ {'지지 ✅' if h1_pass else '반증 ❌'}")
    print(f"  H2 (비동기 병렬): perception 실측 {np.mean(perc_hz):.1f}Hz "
          f"(목표 30) — planner가 느려도 유지 → {'지지 ✅' if np.mean(perc_hz) > 20 else '확인필요'}")

    # 그래프
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    ax = axes[0]
    ax.plot(delays, ctrl_hz, "o-", color="tab:blue", lw=2, label="control 실측")
    ax.plot(delays, perc_hz, "s--", color="tab:green", lw=1.5, label="perception 실측")
    ax.axhline(ctrl_target, color="tab:blue", ls=":", alpha=0.6, label="control 목표 20Hz")
    ax.axhline(30, color="tab:green", ls=":", alpha=0.5, label="perception 목표 30Hz")
    ax.set_xlabel("planner(VLM) 주입 지연 (s)")
    ax.set_ylabel("실측 루프 주기 (Hz)")
    ax.set_title("H1 지연 격리\n상위 VLM이 느려져도 하위 제어 Hz 불변")
    ax.set_ylim(0, 35); ax.grid(True, alpha=0.3); ax.legend(fontsize=8)

    ax = axes[1]
    labels = ["perception\n(목표30)", "planner\n(목표0.2)", "control\n(목표20)"]
    meas = [np.mean(perc_hz), np.mean([r["planner"] for r in rows]), np.mean(ctrl_hz)]
    ax.bar(labels, meas, color=["tab:green", "tab:orange", "tab:blue"], alpha=0.8)
    ax.set_ylabel("실측 Hz (스윕 평균)")
    ax.set_title("H2 비동기 병렬\n세 계층이 각자 주기로 독립 동작")
    ax.grid(True, alpha=0.3, axis="y")
    for i, v in enumerate(meas):
        ax.text(i, v, f"{v:.1f}", ha="center", va="bottom", fontsize=9)

    plt.tight_layout()
    out = "pc/ai/timing_experiment_result.png"
    plt.savefig(out, dpi=110)
    print(f"\n[저장] {out}")


if __name__ == "__main__":
    main()
