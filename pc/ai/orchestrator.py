"""
[하위-PC 계층] 오케스트레이터.
세 계층을 각자 다른 주기의 스레드로 돌리고, 최종 goto 명령을 로봇에 보낸다.

  perception 스레드 (~30Hz) : 최신 Detections 갱신
  planner 스레드    (~0.2Hz) : 최신 Goal 갱신 (VLM은 느리므로 가끔)
  control 스레드    (~20Hz)  : Goal + Detections + pose -> 구체 goto -> 로봇 전송

이 "서로 다른 주기로 도는 여러 스레드"가 곧 다중 프로세싱 제어의 실체다.

실행:
  python pc/ai/orchestrator.py --sim              # 로봇 없이 가짜 pose로 집에서 검증
  python pc/ai/orchestrator.py --host 192.168.43.1 --port 9999   # 실제 로봇
"""

from __future__ import annotations
import argparse
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))          # pc/ai
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # pc

# Windows 콘솔(cp949)에서도 유니코드 출력 가능하게
try:
    sys.stdout.reconfigure(encoding="utf-8")
except AttributeError:
    pass

from perception import MockPerception, OpenCvPerception, Detection   # noqa: E402
from planner import MockPlanner, OllamaVLMPlanner, Goal              # noqa: E402


class SimLink:
    """로봇 없이 아키텍처를 돌리기 위한 가짜 링크. pose를 생성하고 goto를 로그로 남김."""
    def __init__(self):
        self.running = True
        self._t0 = time.time()
        self._lock = threading.Lock()
        self.last_goto = None

    @property
    def latest_pose(self):
        t = time.time() - self._t0
        # 천천히 도는 가짜 로봇
        return {"x": 0.3 * math.cos(0.2 * t), "y": 0.3 * math.sin(0.2 * t), "h": 0.2 * t, "slip": 0.0}

    def goto(self, x, y, h_rad):
        with self._lock:
            self.last_goto = (x, y, h_rad)
        print(f"    [SIM goto] x={x:.2f} y={y:.2f} h={math.degrees(h_rad):.0f}°")


class Orchestrator:
    def __init__(self, link, perception, planner):
        self.link = link
        self.perception = perception
        self.planner = planner

        self._lock = threading.Lock()
        self._detections = []      # 최신 검출
        self._goal = Goal("idle", None, "init")
        self._running = False

    def start(self):
        self._running = True
        threading.Thread(target=self._perception_loop, daemon=True).start()
        threading.Thread(target=self._planner_loop, daemon=True).start()
        threading.Thread(target=self._control_loop, daemon=True).start()

    def stop(self):
        self._running = False

    # --- 중위: 빠른 인식 (~30Hz) ---
    def _perception_loop(self):
        while self._running:
            pose = self._pose_tuple()
            dets = self.perception.detect(frame=None, robot_pose=pose)
            with self._lock:
                self._detections = dets
            time.sleep(1 / 30.0)

    # --- 상위: VLM 판단 (~0.2Hz, 느림) ---
    def _planner_loop(self):
        while self._running:
            with self._lock:
                dets = list(self._detections)
            pose = self._pose_tuple()
            goal = self.planner.plan(dets, pose, frame=None)
            with self._lock:
                self._goal = goal
            print(f"[PLAN] {goal.kind} target={goal.target_label} :: {goal.reason}")
            time.sleep(5.0)   # VLM은 가끔만 (지연 큰 계층)

    # --- 하위: 목표 해석 + 전송 (~20Hz) ---
    def _control_loop(self):
        while self._running:
            with self._lock:
                goal = self._goal
                dets = list(self._detections)
            pose = self._pose_tuple()

            target = self._resolve(goal, dets, pose)
            if target is not None:
                self.link.goto(*target)
            time.sleep(1 / 20.0)

    def _resolve(self, goal: Goal, dets, pose):
        """고수준 Goal + 실시간 Detections -> 구체적 goto(x, y, h). 없으면 None."""
        if goal.kind != "goto_object" or goal.target_label is None:
            return None
        # 목표 라벨 중 로봇에서 가장 가까운 것 선택 (YOLO 좌표 사용)
        cands = [d for d in dets if d.label == goal.target_label]
        if not cands:
            return None
        rx, ry, _ = pose
        nearest = min(cands, key=lambda d: (d.field_x - rx) ** 2 + (d.field_y - ry) ** 2)
        # 목표 물체 위치로 이동. heading은 고정(0)으로 둬 goto 목표가 흔들리지 않게 한다.
        # (매 루프 "바라보는 각" 재계산 시 목표각이 진동 → 로봇 heading 래치가 계속 리셋됨.
        #  실로봇 데모 안정성 우선. 물체 조준이 필요하면 도달 후 별도 회전 단계로 분리 권장.)
        return (nearest.field_x, nearest.field_y, 0.0)

    def _pose_tuple(self):
        p = self.link.latest_pose
        if not p:
            return (0.0, 0.0, 0.0)
        return (p["x"], p["y"], p["h"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", action="store_true", help="로봇 없이 가짜 pose로 실행")
    ap.add_argument("--host", default="192.168.43.1")
    ap.add_argument("--port", type=int, default=9999)
    ap.add_argument("--seconds", type=float, default=0, help="0이면 무한 실행")
    # 실제 AI 켜기 (없으면 Mock)
    ap.add_argument("--real-planner", action="store_true", help="Ollama LLM으로 판단(느림)")
    ap.add_argument("--model", default="qwen2.5-coder:7b", help="Ollama 모델명")
    ap.add_argument("--real-perception", action="store_true", help="OpenCV 색검출(카메라 필요)")
    ap.add_argument("--camera", type=int, default=0, help="카메라 인덱스")
    args = ap.parse_args()

    if args.sim:
        link = SimLink()
    else:
        from comm_client import RobotLink
        link = RobotLink(args.host, args.port)
        link.connect()

    # 인식 계층: 실제(OpenCV 카메라) or Mock
    if args.real_perception:
        perception = OpenCvPerception(camera_index=args.camera)
        print(f"[인식] OpenCV 색검출 (카메라 {args.camera})")
    else:
        perception = MockPerception()
        print("[인식] Mock (고정 red_block @ 0.5,0.3)")

    # 판단 계층: 실제(Ollama) or Mock
    if args.real_planner:
        planner = OllamaVLMPlanner(model=args.model)
        print(f"[판단] Ollama {args.model} — 워밍업 중(모델 로딩)...")
        planner.warmup()
        print("[판단] 워밍업 완료")
    else:
        planner = MockPlanner()
        print("[판단] Mock (규칙기반)")

    orch = Orchestrator(link, perception, planner)
    orch.start()
    print("[오케스트레이터] 3계층 가동 (perception 30Hz / planner / control 20Hz)")

    try:
        if args.seconds > 0:
            time.sleep(args.seconds)
        else:
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        orch.stop()
        print("\n[종료]")


if __name__ == "__main__":
    main()
