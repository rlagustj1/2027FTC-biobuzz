# 협업 브랜치 전략 (gitflow)

이 저장소는 gitflow 방식으로 브랜치를 관리합니다. `master`, `develop` 브랜치는 GitHub에서
직접 push가 막혀 있고, 반드시 Pull Request(PR)를 통해서만 병합할 수 있습니다.

## 브랜치 종류

| 브랜치 | 용도 | 분기 출발점 | 병합 대상 |
|---|---|---|---|
| `master` | 항상 안정적으로 동작하는(대회에 들고 나갈 수 있는) 코드 | - | - |
| `develop` | 팀원들의 작업을 모으는 통합 브랜치 | `master` | - |
| `feature/기능이름` | 새 기능/서브시스템 작업 | `develop` | `develop` |
| `release/버전` | 대회 직전 배포 준비(버그 수정만, 새 기능 금지) | `develop` | `develop` + `master` |
| `hotfix/이름` | `master`에서 발견된 긴급 버그 수정 | `master` | `master` + `develop` |

## 기본 작업 흐름 (feature 브랜치)

1. `develop`을 최신 상태로 받기
   ```
   git checkout develop
   git pull
   ```
2. 새 feature 브랜치 만들기 (이름은 무엇을 하는지 알 수 있게)
   ```
   git checkout -b feature/odometry-tuning
   ```
3. 작업하고 커밋
   ```
   git add <파일>
   git commit -m "설명"
   ```
4. 원격에 push
   ```
   git push -u origin feature/odometry-tuning
   ```
5. GitHub에서 `feature/odometry-tuning` → `develop` 로 Pull Request 생성
6. 리뷰 후 병합(Merge), 병합된 feature 브랜치는 삭제

## release 브랜치 (대회 출전 직전)

```
git checkout develop
git pull
git checkout -b release/2027-week1
```
이 브랜치에서는 버그 수정만 하고, 준비되면 `develop`과 `master` 양쪽으로 PR을 보냅니다.
`master`로 병합할 때는 태그(`git tag v2027.1`)를 남기는 것을 권장합니다.

## hotfix 브랜치 (대회 현장에서 master에 긴급 버그 발견 시)

```
git checkout master
git pull
git checkout -b hotfix/gripper-crash
```
수정 후 `master`와 `develop` 양쪽에 PR로 병합합니다.

## 브랜치 보호 규칙 (이미 적용됨)

- `master`, `develop`에는 직접 `push` 불가 → 반드시 PR 필요
- Force-push, 브랜치 삭제 불가
- 현재 PR 승인(리뷰) 인원수는 0명으로 설정되어 있어(소규모 팀 특성상), PR만 올리면 병합은 가능합니다.
  팀 리뷰 문화가 자리잡으면 GitHub 저장소 Settings → Branches 에서 필요 승인 인원을 늘릴 수 있습니다.

## 커밋 메시지

과거 커밋 로그처럼 "무엇을 했는지"가 한눈에 보이게 간결히 작성합니다.
예: `오도메트리 좌표계 버그 수정`, `PIDF 게인 실기 튜닝 반영`
