"""
색검출 튜닝 도구 — 카메라 화면 + 검출 결과를 실시간으로 보며 HSV 범위를 맞춘다.
실제 블록/조명에서 OpenCvPerception이 잘 검출되게 확인·조정하는 용도.

실행: python pc/ai/color_tune.py            (카메라 0)
      python pc/ai/color_tune.py --camera 1
조작: 트랙바로 빨강/파랑 HSV 조정. 'q' 종료. 콘솔에 검출 좌표 출력.
      맞는 값이 나오면 그 HSV를 perception.py OpenCvPerception 의 RED/BLUE 상수에 반영.
"""
import sys
import argparse
import cv2
import numpy as np

try:
    sys.stdout.reconfigure(encoding="utf-8")
except AttributeError:
    pass


def nothing(x):
    pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"카메라 {args.camera} 못 엶")
        return

    cv2.namedWindow("tune")
    # 기본값: 빨강(두 구간 중 하나) — 필요시 트랙바로 조정
    cv2.createTrackbar("H_lo", "tune", 0, 180, nothing)
    cv2.createTrackbar("H_hi", "tune", 10, 180, nothing)
    cv2.createTrackbar("S_lo", "tune", 120, 255, nothing)
    cv2.createTrackbar("V_lo", "tune", 70, 255, nothing)
    cv2.createTrackbar("MinArea", "tune", 800, 8000, nothing)

    print("트랙바로 HSV 맞추기. 초록 박스=검출. 'q' 종료.")
    print("빨강은 Hue가 0근처+170근처 두 구간임(경계). 파랑은 H 100~130 부근.")

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hlo = cv2.getTrackbarPos("H_lo", "tune")
        hhi = cv2.getTrackbarPos("H_hi", "tune")
        slo = cv2.getTrackbarPos("S_lo", "tune")
        vlo = cv2.getTrackbarPos("V_lo", "tune")
        min_area = cv2.getTrackbarPos("MinArea", "tune")

        mask = cv2.inRange(hsv, np.array([hlo, slo, vlo]), np.array([hhi, 255, 255]))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            if cv2.contourArea(c) < min_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f"{int(cv2.contourArea(c))}", (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        combo = np.hstack([frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
        cv2.putText(combo, f"H[{hlo}-{hhi}] S>{slo} V>{vlo} area>{min_area}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imshow("tune", combo)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            print(f"\n[선택한 HSV] lower=({hlo},{slo},{vlo}) upper=({hhi},255,255) MinArea={min_area}")
            print("이 값을 perception.py OpenCvPerception 의 RED/BLUE 상수에 반영하세요.")
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
