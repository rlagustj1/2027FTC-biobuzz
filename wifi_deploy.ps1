# =============================================================================
# 로봇 WiFi 오프라인 배포 스크립트 (WiFi 어댑터 1개용)
#
# 목적: 인터넷(Claude 대화)이 끊기는 시간을 "배포하는 1~2분"으로 최소화.
#   로봇 WiFi 붙기 → adb 접속(끊기면 재시도) → APK 설치 → 시계보정 → RC앱 실행
#   → [자동으로 집 WiFi 복귀] → 로그 저장.
#
# 실행: PowerShell에서
#   powershell -ExecutionPolicy Bypass -File .\wifi_deploy.ps1
# 끝나면 집 WiFi로 자동 복귀됨. deploy_log.txt 를 Claude에게 보여주면 됨.
# =============================================================================

# --- 설정 (환경 바뀌면 여기만 수정) -----------------------------------------
$ROBOT_WIFI = "FTC-be8o"                 # 로봇 AP SSID
$HOME_WIFI  = "raon_h_202"               # 배포 후 복귀할 집 WiFi
$ROBOT_IP   = "192.168.43.1"
$ADB_PORT   = "5555"
$ADB        = "C:\Users\hyeon\AppData\Local\Android\Sdk\platform-tools\adb.exe"
$APK        = "C:\Users\hyeon\AndroidStudioProjects\2027FTCbiobuzzseason\TeamCode\build\outputs\apk\debug\TeamCode-debug.apk"
$LOG        = "C:\Users\hyeon\AndroidStudioProjects\2027FTCbiobuzzseason\deploy_log.txt"
$RC_PKG     = "com.qualcomm.ftcrobotcontroller"
$RC_ACT     = "org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity"
# ---------------------------------------------------------------------------

# 로그를 파일+화면 동시에
Start-Transcript -Path $LOG -Force | Out-Null
function Log($m) { Write-Host "[$(Get-Date -Format 'HH:mm:ss')] $m" }

$ok = $false
try {
    Log "=== 배포 시작 ==="

    # 1) 로봇 AP로 전환 (부팅 직후면 몇 초 걸릴 수 있어 재시도)
    Log "로봇 WiFi($ROBOT_WIFI) 연결 시도..."
    $connected = $false
    for ($i = 1; $i -le 6; $i++) {
        netsh wlan connect name="$ROBOT_WIFI" | Out-Null
        Start-Sleep -Seconds 5
        if (Test-Connection -ComputerName $ROBOT_IP -Count 1 -Quiet) {
            $connected = $true; Log "로봇 도달 확인 ($ROBOT_IP) — 시도 $i회"; break
        }
        Log "  아직 안 붙음 (시도 $i/6)..."
    }
    if (-not $connected) { throw "로봇 AP에 못 붙음. Control Hub 전원/거리 확인 후 재실행." }

    # 2) 무선 adb 접속 (끊김 대비: disconnect 후 connect, device 상태 확인 재시도)
    Log "무선 adb 접속..."
    $adbReady = $false
    for ($i = 1; $i -le 5; $i++) {
        & $ADB disconnect "$($ROBOT_IP):$ADB_PORT" 2>&1 | Out-Null
        & $ADB connect "$($ROBOT_IP):$ADB_PORT" 2>&1 | Out-Null
        Start-Sleep -Seconds 2
        $dev = & $ADB devices
        if ($dev -match "$ROBOT_IP.*device") { $adbReady = $true; Log "adb 연결됨 — 시도 $i회"; break }
        Log "  adb 아직 offline (시도 $i/5)..."
        Start-Sleep -Seconds 2
    }
    & $ADB devices
    if (-not $adbReady) { throw "adb가 device 상태로 안 붙음. (USB로 한번 'adb tcpip 5555' 필요할 수도)" }

    # 3) APK 설치 (미리 빌드된 것 재설치)
    Log "APK 설치 중... (무선이라 1~2분 걸릴 수 있음)"
    $inst = & $ADB install -r -d "$APK" 2>&1
    $inst | ForEach-Object { Log "  $_" }
    if ($inst -match "Success") { Log "설치 성공"; $ok = $true }
    else { Log "!! 설치 실패 — 위 메시지 확인 (재설치 안 되면 uninstall 후 재시도 필요)" }

    # 4) Control Hub 시계 보정 (부팅 시 1970 리셋 대비)
    Log "시계 보정..."
    $now = Get-Date -Format "MMddHHmmyyyy.ss"   # adb date 형식: MMDDhhmmYYYY.ss
    & $ADB shell "su root date $now" 2>&1 | ForEach-Object { Log "  $_" }
    & $ADB shell "date $now" 2>&1 | ForEach-Object { Log "  $_" }
    Log "  설정 시각: $now"

    # 5) RC 앱 실행
    Log "RC 앱 실행..."
    & $ADB shell am start -n "$RC_PKG/$RC_ACT" 2>&1 | ForEach-Object { Log "  $_" }

    Log "=== 배포 끝 (성공=$ok) ==="
}
catch {
    Log "!! 오류: $_"
}
finally {
    # 6) 무슨 일이 있어도 집 WiFi로 복귀 (Claude 대화 재개용)
    Log "집 WiFi($HOME_WIFI) 복귀 중..."
    netsh wlan connect name="$HOME_WIFI" | Out-Null
    Start-Sleep -Seconds 5
    if (Test-Connection -ComputerName 8.8.8.8 -Count 1 -Quiet) { Log "인터넷 복구됨 — Claude와 다시 대화 가능" }
    else { Log "아직 인터넷 안 붙음. 수동으로 $HOME_WIFI 연결하세요." }
    Log "로그 파일: $LOG"
    Stop-Transcript | Out-Null
}
