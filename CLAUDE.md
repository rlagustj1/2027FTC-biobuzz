# 공학 바이브 코딩 가이드 (FTC 제어공학 적용용)

이 프로젝트는 FTC(FIRST Tech Challenge) 로봇 제어 코드(모션 프로파일링, PIDF, 오도메트리 등)를 다룬다. 웹/앱 코딩과 달리 실기 검증이 필요하고 실패 비용이 높으므로, 코드를 작성/수정할 때 반드시 아래 원칙을 따를 것.

## 핵심 전제: 웹 코딩과 공학 코딩은 검증 방식이 다르다

| 구분 | 웹 바이브 코딩 | 공학 바이브 코딩 |
|---|---|---|
| 검증 방법 | 화면 보고 바로 판단 가능 | 실제로 로봇을 굴려봐야 판단 가능 |
| 실패 비용 | 낮음 (새로고침하면 됨) | 높음 (하드웨어 손상, 안전 문제 가능) |
| 파라미터 | AI가 임의로 채워도 무방 (색상, 폰트 등) | 반드시 실측/튜닝 필요 (vmax, amax, kP/kI/kD) |
| 디버깅 단위 | 전체 페이지 단위로 봐도 됨 | 반드시 작은 단위로 쪼개서 검증 |

## 제어 코드 작업 시: `ftc-control-engineering` 스킬을 따른다

모션 프로파일링, PIDF 컨트롤러, 오도메트리, 자율주행 로직을 작성·수정할 때는
[.claude/skills/ftc-control-engineering/SKILL.md](.claude/skills/ftc-control-engineering/SKILL.md)에
정리된 상세 규칙(파라미터 실측 placeholder, 단위 명시, 단계별 검증, 시뮬레이션 우선,
삼각형 프로파일 분기, 원리 설명, 이론-실측 비교, 체크리스트)을 반드시 따른다. 요약하면:

- 파라미터(VMAX/AMAX/kP/kI/kD 등)는 실측 전까지 "TODO: 실측 필요" 주석과 함께 임시값으로만 넣는다
- 사용자 동의 없이 시뮬레이션 → PIDF → 통합 → Java 이식 → 실기 튜닝의 4단계 이상을 한 번에 진행하지 않는다
- Python 시뮬레이션으로 검증되지 않은 로직은 Java로 이식하지 않는다

## 파일 배치 규칙 (제어 코드 여부와 무관하게 모든 FTC 파일에 적용)

새 FTC(Java) 파일은 반드시 아래 패키지 구조에 맞춰 배치한다. `teamcode/` 루트에 흩뿌리지 않는다. (작년 팀 레포 컨벤션과 동일)

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── opmodes/            OpMode만 모음 (@TeleOp/@Autonomous 붙은 클래스)
│   ├── TeleOp/           수동 조종 OpMode
│   ├── auto/             자율주행 OpMode
│   └── tests/            하드웨어/기능 테스트용 OpMode
├── subsystems/         드라이브트레인·팔 등 하드웨어 서브시스템 (FTCLib SubsystemBase)
├── commands/           동작 단위 Command (FTCLib CommandBase)
├── localization/       오도메트리·야코비안 등 위치추정/기구학
├── comm/               외부 PC 통신 (소켓 서버 등)
└── Utils/              공용 상수·헬퍼
```

원칙:
- **OpMode는 반드시 `opmodes/` 하위**에 두고, 종류별(TeleOp/auto/tests)로 분류한다.
- 로직(subsystem/command/localization)과 진입점(OpMode)을 섞지 않는다.
- 새 폴더가 필요하면 이 구조와 일관되게 소문자 패키지명으로 추가한다.
- 파일을 옮기면 `package` 선언과 import를 반드시 함께 수정하고 빌드로 검증한다.

## 프로젝트 기술 스택

- **공식 FtcRobotController 구조** (SDK 11.0) + FTCLib 2.1.1
- 팀 코드 위치: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`
- 배포: `./gradlew :TeamCode:installDebug` (USB), RC 앱 = `com.qualcomm.ftcrobotcontroller`
- 시뮬레이션: Python (numpy/matplotlib), 검증 후에만 Java로 이식
- 외부 PC: `pc/` (통신 클라이언트, 웹 대시보드, AI 오케스트레이터)
