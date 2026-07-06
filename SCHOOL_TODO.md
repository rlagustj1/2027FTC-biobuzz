# 학교 가서 할 일 (BioBuzz FTC)

집에서 코드는 다 짜뒀고(시뮬 검증 완료), 학교에선 **연결 → 통신 검증 → 오도 실측 → 통합** 순서로 진행.

---

## 0. 연결 세팅 (제일 먼저)

- [ ] **FTC Companion 플러그인 설치**
  - Android Studio → `File → Settings → Plugins → Marketplace` 에서 "FTC Companion" 검색 → Install → 재시작
  - (작년에 쓰던 무선 배포 플러그인. github.com/yoyok978/ftccompanion)
  - ⚠️ 최신 Android Studio 2026.1.1과 호환 안 되면 → 아래 "순정 무선 ADB" 방식 사용
- [ ] **최초 1회 USB 연결** (무선 배포도 처음엔 USB 필요)
  - Control Hub ↔ 노트북 USB-C 데이터 케이블 (충전 전용 X)
  - Control Hub 웹(`192.168.43.1:8080`) → Settings → USB Debugging 켜짐 확인
  - Android Studio 디바이스 목록에 Control Hub 뜨는지 확인
- [ ] 코드 빌드 → Control Hub에 설치 (▶ Run)

### (플러그인 안 되면) 순정 무선 ADB 방식
```
adb tcpip 5555
adb connect 192.168.43.1:5555
```
→ 이후 USB 빼도 Wi-Fi로 배포됨

---

## 1. 통신 검증 (오도 달기 전에 가능 — 하드웨어 불필요)

- [ ] Driver Station에서 `Comm Test (fake pose)` 실행 (INIT → START)
- [ ] 노트북을 Control Hub Wi-Fi에 접속
- [ ] `python pc/dashboard.py` 실행 → 브라우저 `http://localhost:8000`
- [ ] 초록 로봇이 원 그리며 돌면 ✅ pose 스트리밍 OK
- [ ] `ping` 버튼 → **WiFi 왕복지연(RTT) ms 기록** (세특 데이터로 남길 것)
- [ ] 필드 클릭 → DS 텔레메트리에 `last goto` 뜨면 ✅ 양방향 통신 OK

---

## 2. 오도메트리 장착 + 실측 (핵심)

- [ ] 데드휠 2개(parallel, perpendicular) + IMU 장착, 구동모터 엔코더 포트에 결선
- [ ] **아래 실측값 측정 → 다음 세션에 Claude에게 전달** (placeholder 채우는 데 필요):
  - [ ] 오도휠 반지름 (m)
  - [ ] 오도 엔코더 틱/회전
  - [ ] 데드휠 오프셋 PARALLEL_OFFSET, PERP_OFFSET (m)
  - [ ] 구동바퀴 반지름 (m)
  - [ ] 트랙폭(좌우 간격), 휠베이스(앞뒤 간격) → L 계산용
  - [ ] IMU 장착 방향 (로고 방향 / USB 방향)
  - [ ] 엔코더가 결선된 모터 포트 config 이름 (parallel용, perp용)

---

## 3. 오도 확인 OpMode (실측 검증 — CLAUDE.md 규칙 7)

- [ ] `Localizer.java` 파라미터에 실측값 입력
- [ ] 로봇을 **손으로 밀면서** 텔레메트리 pose(x, y, θ)가 실제 이동과 맞는지 확인:
  - [ ] 앞으로 1m 밀기 → x가 1.0 근처?
  - [ ] 옆으로 밀기 → y 변화, x 유지?
  - [ ] 제자리 회전 → θ만 변하고 x, y 유지?
- [ ] 부호/방향 틀리면 조정 (모터 방향, 오프셋 부호, IMU 방향)

---

## 4. 통합 (통신 + 오도)

- [ ] `CommTestOpMode`의 가짜 pose를 실제 `Localizer`로 교체
- [ ] 대시보드에 **진짜 로봇 위치** 실시간 표시 확인
- [ ] 밀었을 때 대시보드 로봇 점이 실제와 같이 움직이면 ✅

---

## 우선순위 메모
- 0 → 1(통신)까지는 오도 없이 바로 됨. 도착하자마자 진행.
- 오도 장착(2)은 시간 걸림 → 그다음.
- 3~4는 오도 실측값 나와야 진행 가능.

## 다음 세션에 Claude에게 줄 것
→ **2번 실측값들.** 그거 받으면 Localizer/MecanumJacobian placeholder 다 채우고 통합 마무리.
