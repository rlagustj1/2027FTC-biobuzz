"""
[상위 계층] VLM 전략 판단 (Planner).
장면/검출/pose를 보고 "무엇을 할지"라는 고수준 Goal을 (가끔) 결정한다.
느려도 되지만 똑똑한 계층 — Ollama 비전 모델이 들어갈 자리.

MockPlanner로 아키텍처를 먼저 돌리고, OllamaVLMPlanner는 비전 모델을 받은 뒤 활성화.
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import List, Optional
from perception import Detection  # 같은 폴더 실행 기준


@dataclass
class Goal:
    """상위 계층이 내린 고수준 목표."""
    kind: str                 # "goto_object" | "idle" 등
    target_label: Optional[str]   # 예: "red_block"
    reason: str               # 왜 이 목표를 골랐는지 (VLM 설명 — 세특 근거로 유용)


class PlannerBase:
    def plan(self, detections: List[Detection], robot_pose, frame=None) -> Goal:
        raise NotImplementedError


class MockPlanner(PlannerBase):
    """
    VLM 없이 규칙기반으로 흉내내는 플래너.
    "빨간 블록이 보이면 그리로 가라"는 단순 전략.
    실제 VLM은 이 자리에서 장면을 해석해 더 복잡한 판단을 한다.
    """
    def plan(self, detections: List[Detection], robot_pose, frame=None) -> Goal:
        reds = [d for d in detections if d.label == "red_block"]
        if reds:
            return Goal("goto_object", "red_block",
                        "빨간 블록이 감지됨 → 수거 대상으로 선택 (mock 규칙)")
        return Goal("idle", None, "대상 없음 → 대기")


# --- 실제 Ollama VLM 자리 (비전 모델 받은 뒤 활성화) --------------------------
class OllamaVLMPlanner(PlannerBase):
    """
    Ollama 로컬 VLM에 장면 이미지를 보내 고수준 전략을 받는 플래너.
    사용 전: `ollama pull llava`  (또는 qwen2-vl 등 비전 지원 모델)
    """
    def __init__(self, model: str = "llava", host: str = "http://localhost:11434"):
        self.model = model
        self.host = host

    def plan(self, detections: List[Detection], robot_pose, frame=None) -> Goal:
        import json
        import urllib.request

        # 검출 요약을 텍스트로 만들어 프롬프트에 포함 (프레임 이미지가 있으면 함께 전송 가능)
        det_summary = ", ".join(f"{d.label}@({d.field_x:.2f},{d.field_y:.2f})" for d in detections)
        prompt = (
            "너는 FTC 로봇의 전략 판단 모듈이다. "
            f"현재 로봇 pose={robot_pose}, 감지된 물체=[{det_summary}]. "
            "다음 목표를 JSON으로만 답하라: "
            '{"kind":"goto_object","target_label":"<라벨>","reason":"<이유>"}'
        )
        payload = {"model": self.model, "prompt": prompt, "stream": False}
        # 이미지가 있으면 base64로 넣어 진짜 VLM 추론 가능:
        # payload["images"] = [base64.b64encode(jpeg_bytes).decode()]

        try:
            req = urllib.request.Request(
                f"{self.host}/api/generate",
                data=json.dumps(payload).encode("utf-8"),
                headers={"Content-Type": "application/json"},
            )
            with urllib.request.urlopen(req, timeout=30) as resp:
                out = json.loads(resp.read().decode("utf-8"))
            text = out.get("response", "")
            g = json.loads(text)  # 모델이 JSON만 뱉는다는 가정 (실전엔 파싱 방어 필요)
            return Goal(g.get("kind", "idle"), g.get("target_label"), g.get("reason", ""))
        except Exception as e:
            # VLM 실패 시 안전하게 대기 (제어는 로봇 로컬이 계속 담당하므로 위험 없음)
            return Goal("idle", None, f"VLM 호출 실패: {e}")
