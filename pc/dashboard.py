"""
웹 대시보드 브리지 (외부 PC에서 실행, Python 표준 라이브러리만 사용).

구조:
  브라우저 --HTTP--> 이 브리지 --TCP--> Control Hub(RobotCommServer)

역할:
  - Control Hub의 RobotCommServer에 TCP로 접속 (comm_client.RobotLink 재사용)
  - 브라우저에 dashboard.html 서빙
  - /state (최신 pose+RTT JSON), /goto, /ping 엔드포인트 제공

사용법 (Control Hub WiFi에 접속하고 CommTestOpMode 실행 후):
  python pc/dashboard.py
  python pc/dashboard.py --robot-host 192.168.43.1 --robot-port 9999 --web-port 8000
그다음 브라우저에서 http://localhost:8000 접속.
"""

import argparse
import csv
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

from comm_client import RobotLink

HTML_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dashboard.html")

LINK = None          # type: RobotLink
LAST_CONNECT_TRY = 0.0
# 최신 goto 목표 (tx[m], ty[m], th[rad]) — CSV 로깅에서 목표-실측 비교용. None이면 목표 없음.
LATEST_TARGET = None


def pose_logger(path, hz=20.0):
    """
    목표(goto)와 실측 pose를 CSV로 기록 (보고서 Ⅲ.3 궤적 비교 데이터).
    컬럼: t[s], target_x, target_y, target_h_deg, x, y, h_deg
    로봇 불필요 — 대시보드가 받는 pose 스트림을 그대로 저장한다.
    """
    period = 1.0 / hz
    t0 = time.time()
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "target_x", "target_y", "target_h_deg", "x", "y", "h_deg"])
        while True:
            with LINK._lock:
                pose = LINK.latest_pose
            tgt = LATEST_TARGET
            if pose is not None:
                w.writerow([
                    round(time.time() - t0, 3),
                    "" if tgt is None else round(tgt[0], 4),
                    "" if tgt is None else round(tgt[1], 4),
                    "" if tgt is None else round(math.degrees(tgt[2]), 2),
                    round(pose.get("x", 0.0), 4),
                    round(pose.get("y", 0.0), 4),
                    round(math.degrees(pose.get("h", 0.0)), 2),
                ])
                f.flush()
            time.sleep(period)


def ensure_connected():
    """로봇 링크가 끊겨 있으면 주기적으로 재접속 시도 (백그라운드)."""
    global LAST_CONNECT_TRY
    while True:
        if LINK is not None and not LINK.running:
            now = time.time()
            if now - LAST_CONNECT_TRY > 2.0:
                LAST_CONNECT_TRY = now
                try:
                    LINK.connect()
                except OSError:
                    pass
        time.sleep(0.5)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass  # 콘솔 스팸 방지

    def _send_json(self, obj, code=200):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_html(self):
        try:
            with open(HTML_PATH, "rb") as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except OSError:
            self.send_error(404, "dashboard.html not found")

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/" or path == "/index.html":
            self._send_html()

        elif path == "/state":
            with LINK._lock:
                pose = LINK.latest_pose
            self._send_json({
                "connected": bool(LINK.running),
                "pose": pose,
                "rtt": LINK.last_rtt_ms,
            })

        elif path == "/ping":
            if LINK.running:
                try:
                    LINK.ping()
                except OSError:
                    pass
            self._send_json({"ok": True})

        elif path == "/goto":
            q = parse_qs(parsed.query)
            try:
                global LATEST_TARGET
                x = float(q["x"][0]); y = float(q["y"][0]); hdeg = float(q["h"][0])
                hrad = hdeg * 3.141592653589793 / 180.0
                LATEST_TARGET = (x, y, hrad)   # CSV 로깅용 최신 목표 기록
                if LINK.running:
                    LINK.goto(x, y, hrad)
                self._send_json({"ok": True, "x": x, "y": y, "h_deg": hdeg})
            except (KeyError, ValueError, OSError) as e:
                self._send_json({"ok": False, "error": str(e)}, code=400)

        else:
            self.send_error(404)


def main():
    global LINK
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-host", default="192.168.43.1")
    ap.add_argument("--robot-port", type=int, default=9999)
    ap.add_argument("--web-port", type=int, default=8000)
    ap.add_argument("--log", default=None,
                    help="pose/목표를 이 CSV 경로에 기록 (보고서 Ⅲ.3 데이터). 예: --log runs/run1.csv")
    args = ap.parse_args()

    LINK = RobotLink(args.robot_host, args.robot_port)
    # 최초 접속 시도 (실패해도 백그라운드가 재시도)
    try:
        LINK.connect()
    except OSError:
        print(f"[대기] 로봇({args.robot_host}:{args.robot_port}) 아직 미연결 — 재시도 계속함")

    threading.Thread(target=ensure_connected, daemon=True).start()

    if args.log:
        os.makedirs(os.path.dirname(os.path.abspath(args.log)), exist_ok=True)
        threading.Thread(target=pose_logger, args=(args.log,), daemon=True).start()
        print(f"[로깅] 목표+실측 pose → {args.log}")

    server = ThreadingHTTPServer(("0.0.0.0", args.web_port), Handler)
    print(f"[대시보드] 브라우저에서 http://localhost:{args.web_port} 접속")
    print(f"[로봇 링크] {args.robot_host}:{args.robot_port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        LINK.close()
        server.shutdown()
        print("\n[종료]")


if __name__ == "__main__":
    main()
