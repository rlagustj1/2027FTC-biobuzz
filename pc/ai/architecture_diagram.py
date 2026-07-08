"""
자율주행 AI 아키텍처 다이어그램 생성 (보고서 Ⅳ. 시스템 설계 그림).

3계층 다중 프로세싱 구조를 지연-정밀도 축과 함께 시각화한다:
  외부 PC(상위 VLM / 중위 인식 / 하위 오케스트레이터) ── TCP+JSON ── Control Hub(실시간 제어)

출력: pc/ai/architecture_diagram.png
로봇 불필요 (그림만 그림).
"""

import sys
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

try:
    sys.stdout.reconfigure(encoding="utf-8")
except AttributeError:
    pass

plt.rcParams["font.family"] = "Malgun Gothic"
plt.rcParams["axes.unicode_minus"] = False

fig, ax = plt.subplots(figsize=(12, 9))
ax.set_xlim(0, 12)
ax.set_ylim(0, 12)
ax.axis("off")


def box(x, y, w, h, title, lines, face, edge, tsize=11):
    """둥근 박스 + 제목 + 설명줄."""
    ax.add_patch(FancyBboxPatch(
        (x, y), w, h, boxstyle="round,pad=0.08,rounding_size=0.12",
        facecolor=face, edgecolor=edge, linewidth=2, zorder=2))
    ax.text(x + 0.25, y + h - 0.33, title, fontsize=tsize, fontweight="bold",
            color=edge, zorder=3)
    for i, ln in enumerate(lines):
        ax.text(x + 0.25, y + h - 0.72 - i * 0.36, ln, fontsize=9, color="#222", zorder=3)


def down_arrow(x, y0, y1, label):
    ax.add_patch(FancyArrowPatch((x, y0), (x, y1), arrowstyle="-|>", mutation_scale=18,
                                 color="#444", linewidth=1.6, zorder=1))
    ax.text(x + 0.15, (y0 + y1) / 2, label, fontsize=8.5, color="#444",
            va="center", ha="left", style="italic", zorder=3)


# ===== 외부 PC 컨테이너 =====
ax.add_patch(FancyBboxPatch((0.4, 5.2), 8.4, 6.5,
             boxstyle="round,pad=0.1,rounding_size=0.15",
             facecolor="#f2f5fa", edgecolor="#9aa7b8", linewidth=1.5,
             linestyle="--", zorder=0))
ax.text(0.65, 11.35, "외부 PC (offboard compute · 고성능)", fontsize=12,
        fontweight="bold", color="#5a6675")

# 상위: VLM Planner
box(0.9, 9.55, 7.4, 1.55, "① 상위 — 전략 판단 (VLM Planner)",
    ["\"무엇을 할지\"  예: \"가장 가까운 빨간 블록으로 가라\"",
     "Ollama(llava/qwen-vl)  ·  API(HTTP /api/generate)  ·  ~0.2 Hz (가끔)"],
    "#e8def8", "#7b4fbf")
down_arrow(4.6, 9.5, 8.75, "Goal(전략, 서술적)")

# 중위: Perception
box(0.9, 7.75, 7.4, 1.55, "② 중위 — 인식 (Perception)",
    ["\"어디 있는지\"  예: red_block @ field(1.2, -0.4)",
     "YOLO(CNN) / OpenCV 색검출  ·  ~30 Hz  ·  정확한 좌표+신뢰도"],
    "#d7ece0", "#2f8f5f")
down_arrow(4.6, 7.7, 6.95, "Detections(좌표)")

# 하위-PC: Orchestrator
box(0.9, 5.55, 7.4, 1.65, "③ 하위(PC) — 오케스트레이터",
    ["Goal + Detections + pose  →  구체적 goto(x, y, h)",
     "서로 다른 주기의 3스레드 관리  ·  ~20 Hz  ·  결정적 목표 생성"],
    "#dce6f5", "#3568b5")

# ===== 통신 채널 =====
ax.add_patch(FancyArrowPatch((3.9, 5.5), (3.9, 3.55), arrowstyle="-|>",
             mutation_scale=20, color="#c0392b", linewidth=2.2, zorder=1))
ax.text(3.55, 4.5, "goto ▼", fontsize=9.5, color="#c0392b", ha="right", fontweight="bold")
ax.add_patch(FancyArrowPatch((5.3, 3.55), (5.3, 5.5), arrowstyle="-|>",
             mutation_scale=20, color="#2471a3", linewidth=2.2, zorder=1))
ax.text(5.65, 4.5, "▲ pose / RTT", fontsize=9.5, color="#2471a3", ha="left", fontweight="bold")
ax.text(4.6, 4.5, "TCP+JSON\n:9999", fontsize=8.5, color="#555", ha="center",
        va="center", bbox=dict(boxstyle="round", fc="white", ec="#aaa"))

# ===== Control Hub =====
ax.add_patch(FancyBboxPatch((0.4, 0.6), 8.4, 2.85,
             boxstyle="round,pad=0.1,rounding_size=0.15",
             facecolor="#fdf3e3", edgecolor="#c99a4a", linewidth=1.5, zorder=0))
ax.text(0.65, 3.15, "Control Hub (로봇 온보드 · 실시간)", fontsize=12,
        fontweight="bold", color="#a9772f")
box(0.9, 0.85, 7.4, 1.75, "④ 실시간 제어 (수식만, 모델 없음)",
    ["오도메트리(Localizer) + 경로추종(P제어) + 메카넘 역기구학 → 모터출력",
     "RobotCommServer: pose 송신 / goto 수신  ·  50~1000 Hz  ·  1 ms 이하 반응"],
    "#fbe6c8", "#c07f2a")

# ===== 오른쪽: 지연-정밀도 축 =====
ax.add_patch(FancyArrowPatch((10.4, 11.0), (10.4, 1.2), arrowstyle="-|>",
             mutation_scale=22, color="#333", linewidth=2))
ax.text(10.75, 10.3, "느림 · 똑똑함", fontsize=10, rotation=90, va="center", color="#7b4fbf")
ax.text(10.75, 6.0, "지연 ↓  정밀도/속도 ↑", fontsize=10, rotation=90, va="center",
        color="#555", fontweight="bold")
ax.text(10.75, 1.9, "빠름 · 정확함", fontsize=10, rotation=90, va="center", color="#c07f2a")
ax.text(9.5, 10.6, "초 단위\nOK", fontsize=8, ha="center", color="#7b4fbf")
ax.text(9.5, 8.5, "수십 ms", fontsize=8, ha="center", color="#2f8f5f")
ax.text(9.5, 1.7, "1 ms\n이하", fontsize=8, ha="center", color="#c07f2a")

ax.text(6.0, 11.9, "다중 프로세싱 계층 제어 아키텍처",
        fontsize=15, fontweight="bold", ha="center")

plt.tight_layout()
out = "pc/ai/architecture_diagram.png"
plt.savefig(out, dpi=120, bbox_inches="tight")
print(f"[저장] {out}")
