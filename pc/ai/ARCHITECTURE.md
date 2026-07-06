# 3계층 다중 프로세싱 제어 아키텍처

VLM + 빠른 비전 + 실시간 제어를 계층으로 나눈 offboard compute 구조.
핵심 원리: **각 계층은 지연-정밀도 트레이드오프가 다르므로, 서로 다른 종류의 모델/연산을 쓰고 서로 다른 주기로 돈다.**

```
┌─────────────────────────────────────────────────────────────┐
│  외부 PC (고성능)                                             │
│                                                               │
│   [상위] VLM Planner        느림·똑똑함    ~0.2Hz (가끔)      │
│      "무엇을 할지" 전략      Ollama(llava/qwen-vl)            │
│      예: "가장 가까운 빨간 블록으로 가라"                     │
│           │ Goal(semantic)                                    │
│           ▼                                                   │
│   [중위] Perception (YOLO)  빠름·정확함    ~30Hz              │
│      "그게 어디 있는지" 좌표                                  │
│      예: red_block @ field(1.2, -0.4)                         │
│           │ Detections                                        │
│           ▼                                                   │
│   [하위-PC] Orchestrator    빠름          ~20Hz              │
│      Goal + Detections + pose → 구체적 goto(x,y,h)           │
│           │                                                   │
└───────────┼──────────────── TCP+JSON ────────────────────────┘
            ▼ goto / ▲ pose
┌───────────────────────────────────────────────────────────────┐
│  Control Hub                                                   │
│   [실시간 제어]  수식만(모델X)   초고속   50~1000Hz           │
│      오도메트리(Localizer) + 경로추종 + 모터출력              │
│      RobotCommServer 로 pose 송신 / goto 수신                 │
└───────────────────────────────────────────────────────────────┘
```

## 왜 이렇게 나누나 (세특 핵심 논지)

| 계층 | 모델 | 주기 | 지연 허용 | 출력 성격 |
|---|---|---|---|---|
| VLM Planner | 언어-비전 모델 | 가끔 | 초 단위 OK | 애매/서술적 전략 |
| Perception | CNN(YOLO) | 30~60Hz | 수십 ms | 정확한 좌표+신뢰도 |
| 실시간 제어 | 수식(PID/기구학) | 1kHz | 1ms 이하 | 결정적 모터출력 |

- **VLM을 제어루프에 직접 못 쓰는 이유**: 추론이 수백 ms~초라 1kHz 루프를 못 따라감 + 출력이 비결정적.
- **제어를 PC에 못 두는 이유**: WiFi 왕복지연이 모터 안정성을 해침 → 실시간 제어는 반드시 Control Hub 로컬.
- 그래서 **느려도 되는 판단은 위로, 빨라야 하는 제어는 아래로** 분리.

## 통신
- 로봇 ↔ PC: 이미 만든 TCP+JSON 채널 하나 (`RobotCommServer` ↔ `RobotLink`).
- VLM(Ollama), YOLO는 전부 PC 내부에서 Python이 호출하는 로컬 서비스. 로봇은 관여 안 함.
- 즉 **로봇 프로토콜은 그대로**, 지능은 전부 PC쪽에 얹힘.

## 파일 구성 (pc/ai/)
- `perception.py` — Detection 자료형 + PerceptionBase 인터페이스 + MockPerception (+ YOLO 자리)
- `planner.py`    — Goal 자료형 + PlannerBase 인터페이스 + MockPlanner (+ Ollama VLM 자리)
- `orchestrator.py` — 세 계층을 각자 스레드로 돌리고 goto를 로봇에 전송

## 현재 상태 / 다음
- 지금: 인터페이스 + Mock으로 **아키텍처 골격 완성, 집에서 실행 가능**.
- 나중에 끼울 것:
  - Perception: `ollama pull` 대신 YOLO(ultralytics) 또는 OpenCV 색검출로 MockPerception 교체
  - Planner: Ollama에 비전 모델 받아서(`ollama pull llava` 등) OllamaVLMPlanner 활성화
  - 카메라 프레임: 로봇캠 스트리밍 or PC 웹캠 (별도 통합 단계)
```
