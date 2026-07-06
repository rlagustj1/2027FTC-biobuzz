# 2027 FTC BioBuzz Season

FTC(FIRST Tech Challenge) 로봇 제어 프로젝트 + 제어공학 세특 탐구.
메카넘 드라이브 · 오도메트리 · 야코비안 · offboard AI(VLM) 다중 프로세싱 제어.

## 구조

```
app/                          FTC SDK 로봇 코드 (Android Studio / Gradle)
  src/main/java/org/firstinspires/ftc/teamcode/
    localization/             오도메트리 + 야코비안
      Localizer.java            2륜 데드휠 + IMU 오도메트리
      MecanumJacobian.java      역기구학 / 유사역행렬 순기구학 / 영공간 슬립검출
    comm/RobotCommServer.java   Control Hub TCP+JSON 통신 서버
    subsystems/DriveSubsystem.java   FTCLib 드라이브 서브시스템
    commands/DriveDistanceCommand.java
    CommTestOpMode.java         통신 검증용 (가짜 pose 스트리밍)
    EncoderMove(Auto)Test.java  엔코더 이동 테스트

sim/                          Python 시뮬레이션 (검증 후 Java 이식)
  odometry_ik_sim.py            역기구학 + 오도메트리 + 경로추종
  jacobian_experiment.py        유사역행렬 / 영공간 슬립 / 조작성 타원체

pc/                           외부 PC (offboard compute)
  comm_client.py               로봇 TCP 클라이언트 (pose 수신 / RTT / goto)
  dashboard.py + dashboard.html  웹 대시보드 (필드뷰, 클릭 goto)
  ai/                          3계층 AI 아키텍처
    ARCHITECTURE.md              VLM / YOLO / 실시간제어 계층 설계
    perception.py, planner.py, orchestrator.py
```

## 3계층 다중 프로세싱 제어

```
VLM Planner (느림·전략)  → YOLO Perception (빠름·좌표)  → 실시간 제어 (초고속·모터)
   외부 PC ─────────── TCP+JSON ─────────── Control Hub
```
자세한 내용: [pc/ai/ARCHITECTURE.md](pc/ai/ARCHITECTURE.md)

## 기술 스택
- FTC SDK 10.1.1 + FTCLib 2.1.1 (Java 11)
- Python (numpy/matplotlib) 시뮬레이션
- Ollama(VLM), YOLO 는 인터페이스만 — 실제 모델은 추후 연결

## 개발 원칙
[CLAUDE.md](CLAUDE.md) — 공학 코딩 가이드(파라미터 실측, 단위 명시, 시뮬 우선, 단계별 검증).
학교 진행 체크리스트: [SCHOOL_TODO.md](SCHOOL_TODO.md)
