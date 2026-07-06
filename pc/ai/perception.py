"""
[중위 계층] 빠른 비전 인식 (Perception).
카메라 프레임 -> 물체 검출(라벨 + 필드좌표). YOLO/OpenCV가 들어갈 자리.

인터페이스만 정의하고 MockPerception으로 아키텍처를 먼저 돌린다.
실제 모델은 PerceptionBase를 상속해 detect()만 구현하면 교체된다.
"""

from __future__ import annotations
import math
import time
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class Detection:
    """검출된 물체 하나. 좌표는 필드 기준(m), 로봇 카메라 보정 후 값이라고 가정."""
    label: str          # 예: "red_block", "blue_block"
    confidence: float   # 0.0 ~ 1.0
    field_x: float      # m (필드 X, +forward)
    field_y: float      # m (필드 Y, +left)


class PerceptionBase:
    """모든 비전 모듈의 공통 인터페이스."""
    def detect(self, frame=None, robot_pose=None) -> List[Detection]:
        """
        :param frame: 카메라 이미지 (numpy 배열 등). Mock은 사용 안 함.
        :param robot_pose: (x, y, h) 현재 로봇 pose — 카메라 상대좌표를 필드좌표로 변환할 때 필요
        :return: Detection 리스트
        """
        raise NotImplementedError


class MockPerception(PerceptionBase):
    """
    실제 카메라/모델 없이 아키텍처를 돌리기 위한 가짜 인식기.
    필드에 고정된 물체 몇 개가 있다고 가정하고, 살짝 노이즈를 섞어 반환한다.
    """
    def __init__(self):
        # 필드에 놓인 가짜 물체들 (label, x, y)
        self._world = [
            ("red_block", 1.2, -0.4),
            ("red_block", 0.6, 0.8),
            ("blue_block", -0.5, 0.3),
        ]

    def detect(self, frame=None, robot_pose=None) -> List[Detection]:
        dets = []
        for label, x, y in self._world:
            # 실제 검출처럼 약간의 좌표 노이즈 + 신뢰도
            nx = x + math.sin(time.time() * 2 + x) * 0.01
            ny = y + math.cos(time.time() * 2 + y) * 0.01
            dets.append(Detection(label, 0.9, nx, ny))
        return dets


# --- 실제 YOLO 자리 (나중에 활성화) -------------------------------------------
# class YoloPerception(PerceptionBase):
#     def __init__(self, weights="best.pt"):
#         from ultralytics import YOLO   # pip install ultralytics
#         self.model = YOLO(weights)
#     def detect(self, frame=None, robot_pose=None):
#         results = self.model(frame)
#         # TODO: 픽셀 bbox -> 카메라 상대좌표 -> robot_pose로 필드좌표 변환 (카메라 보정 필요)
#         ...
