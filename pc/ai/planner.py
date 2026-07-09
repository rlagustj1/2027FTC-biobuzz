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
    def __init__(self, model: str = "llava", host: str = "http://localhost:11434",
                 timeout: float = 120.0):
        self.model = model
        self.host = host
        self.timeout = timeout   # CPU 추론은 느리므로 넉넉히(특히 콜드스타트). VLM은 0.2Hz 계층이라 OK.

    def warmup(self):
        """모델을 미리 메모리에 올려 첫 추론 지연(콜드스타트)을 없앤다."""
        try:
            self.plan([], (0.0, 0.0, 0.0))
        except Exception:
            pass

    def plan(self, detections: List[Detection], robot_pose, frame=None) -> Goal:
        import json
        import re
        import urllib.request

        # 검출 요약을 텍스트로 만들어 프롬프트에 포함 (프레임 이미지가 있으면 함께 전송 가능)
        det_summary = ", ".join(f"{d.label}@({d.field_x:.2f},{d.field_y:.2f})" for d in detections) or "없음"
        prompt = (
            "너는 FTC 로봇의 전략 판단 모듈이다. "
            f"현재 로봇 pose={robot_pose}, 감지된 물체=[{det_summary}]. "
            "가장 적절한 다음 목표를 JSON으로만 답하라(설명 금지). "
            '형식: {"kind":"goto_object","target_label":"<라벨>","reason":"<이유>"} '
            '감지된 물체가 없으면 {"kind":"idle","target_label":null,"reason":"대상 없음"}.'
        )
        payload = {"model": self.model, "prompt": prompt, "stream": False,
                   "format": "json"}   # Ollama JSON 모드: 유효한 JSON만 반환 강제

        try:
            req = urllib.request.Request(
                f"{self.host}/api/generate",
                data=json.dumps(payload).encode("utf-8"),
                headers={"Content-Type": "application/json"},
            )
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                out = json.loads(resp.read().decode("utf-8"))
            text = out.get("response", "").strip()
            # 방어적 파싱: 본문에서 첫 JSON 객체 추출
            m = re.search(r"\{.*\}", text, re.DOTALL)
            g = json.loads(m.group(0) if m else text)
            return Goal(g.get("kind", "idle"), g.get("target_label"), g.get("reason", ""))
        except Exception as e:
            # VLM 실패 시 안전하게 대기 (제어는 로봇 로컬이 계속 담당하므로 위험 없음)
            return Goal("idle", None, f"VLM 호출 실패: {e}")
