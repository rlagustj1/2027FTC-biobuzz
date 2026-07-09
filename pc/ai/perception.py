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
    def __init__(self, demo_stable=True):
        # 필드에 놓인 가짜 물체들 (label, x, y).
        # demo_stable=True: 실로봇 데모용으로 가깝고 안정적인 단일 목표(노이즈 없음).
        self.demo_stable = demo_stable
        if demo_stable:
            self._world = [("red_block", 0.5, 0.3)]   # 안전한 근거리 목표
        else:
            self._world = [
                ("red_block", 1.2, -0.4),
                ("red_block", 0.6, 0.8),
                ("blue_block", -0.5, 0.3),
            ]

    def detect(self, frame=None, robot_pose=None) -> List[Detection]:
        dets = []
        for label, x, y in self._world:
            if self.demo_stable:
                # 데모: 고정 좌표(노이즈 없음) — goto 목표가 흔들리지 않게
                dets.append(Detection(label, 0.95, x, y))
            else:
                nx = x + math.sin(time.time() * 2 + x) * 0.01
                ny = y + math.cos(time.time() * 2 + y) * 0.01
                dets.append(Detection(label, 0.9, nx, ny))
        return dets


# --- 실제 인식: OpenCV 색검출 (빨강/파랑 블록) ---------------------------------
class OpenCvPerception(PerceptionBase):
    """
    웹캠/로봇캠 프레임에서 색(HSV)으로 빨강·파랑 블록을 검출한다.
    학습 불필요, CPU에서 빠름 → FTC 색상 게임요소에 실전적.

    ⚠️ 픽셀→필드좌표 변환은 카메라 보정이 필요하다. 여기서는 화면 하단 중앙을 로봇 전방으로
       보는 '간이 선형 근사'를 쓴다(TODO: 호모그래피/카메라 캘리브레이션으로 교체).
    사용 전: pip install opencv-python
    """
    # HSV 색범위 (조명 따라 튜닝 필요)
    RED1 = ((0, 120, 70), (10, 255, 255))
    RED2 = ((170, 120, 70), (180, 255, 255))   # 빨강은 Hue 경계라 두 구간
    BLUE = ((100, 120, 70), (130, 255, 255))
    MIN_AREA = 800   # 픽셀. 이보다 작은 blob은 노이즈로 무시

    def __init__(self, camera_index=0, frame_w=640, frame_h=480,
                 m_per_px=0.0025, cam_forward_offset=0.15):
        import cv2
        self._cv2 = cv2
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_h)
        self.w, self.h = frame_w, frame_h
        # TODO: 실측 캘리브레이션 — 화면 픽셀당 몇 m인지, 카메라가 로봇 중심에서 앞으로 얼마인지
        self.m_per_px = m_per_px              # 픽셀→미터 (임시, 실측 필요)
        self.cam_forward_offset = cam_forward_offset  # 카메라~로봇중심 전방거리 (임시)

    def _pixel_to_field(self, px, py, robot_pose):
        """(간이) 픽셀 좌표 → 필드 좌표. TODO: 호모그래피로 교체."""
        import math
        rx, ry, rh = robot_pose
        # 화면 하단 중앙 기준: 위로 갈수록 로봇 전방(+x_local), 좌우는 y_local
        dx_local = self.cam_forward_offset + (self.h - py) * self.m_per_px
        dy_local = (self.w / 2 - px) * self.m_per_px   # 화면 왼쪽 = 로봇 왼쪽(+y)
        # 로봇 로컬 → 필드 (heading 회전)
        fx = rx + dx_local * math.cos(rh) - dy_local * math.sin(rh)
        fy = ry + dx_local * math.sin(rh) + dy_local * math.cos(rh)
        return fx, fy

    def _find(self, hsv, label, ranges, robot_pose):
        cv2 = self._cv2
        import numpy as np
        mask = None
        for lo, hi in ranges:
            m = cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for c in cnts:
            area = cv2.contourArea(c)
            if area < self.MIN_AREA:
                continue
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            px, py = M["m10"] / M["m00"], M["m01"] / M["m00"]
            fx, fy = self._pixel_to_field(px, py, robot_pose)
            conf = min(1.0, area / (self.w * self.h * 0.1))
            out.append(Detection(label, round(conf, 2), round(fx, 3), round(fy, 3)))
        return out

    def detect(self, frame=None, robot_pose=None) -> List[Detection]:
        cv2 = self._cv2
        pose = robot_pose or (0.0, 0.0, 0.0)
        if frame is None:
            ok, frame = self.cap.read()
            if not ok:
                return []
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        dets = []
        dets += self._find(hsv, "red_block", [self.RED1, self.RED2], pose)
        dets += self._find(hsv, "blue_block", [self.BLUE], pose)
        return dets

    def release(self):
        try:
            self.cap.release()
        except Exception:
            pass
