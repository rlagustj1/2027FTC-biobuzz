# =============================================================================
# 실제 AI 자율주행 데모 — 원클릭 실행
# GPU 가속 켠 뒤 이 스크립트만 실행하면 됨 (Ollama가 GPU 자동 사용).
#
# 실행: powershell -ExecutionPolicy Bypass -File pc\ai\run_ai_demo.ps1
# 중지: 이 창에서 Ctrl+C  (또는 DS에서 OpMode STOP)
# =============================================================================

# --- 설정 (환경 맞게 수정) --------------------------------------------------
$ROBOT_HOST = "192.168.43.1"      # 무선(FTC-be8o). USB면 "127.0.0.1" + 아래 USE_USB=$true
$USE_USB    = $false              # USB로 할 거면 $true (adb forward 자동)
$MODEL      = "qwen2.5-coder:7b"  # Ollama 모델 (GPU면 빠름)
$CAMERA     = 0                   # 카메라 인덱스
$USE_CAMERA = $true               # $false면 Mock 인식(고정 red_block)
# ---------------------------------------------------------------------------

$PROJ = "C:\Users\hyeon\AndroidStudioProjects\2027FTCbiobuzzseason"
$ADB  = "C:\Users\hyeon\AppData\Local\Android\Sdk\platform-tools\adb.exe"
Set-Location $PROJ

if ($USE_USB) {
    $ROBOT_HOST = "127.0.0.1"
    Write-Host "[USB] adb forward 설정..."
    & $ADB forward tcp:9999 tcp:9999 | Out-Null
}

# GPU 사용 여부 표시 (참고용)
Write-Host "=== Ollama GPU 상태 ==="
try { & ollama ps } catch {}

$percFlag = if ($USE_CAMERA) { "--real-perception" } else { "" }
Write-Host "=== 자율주행 데모 시작 (로봇 $ROBOT_HOST / 모델 $MODEL / 카메라 $USE_CAMERA) ==="
Write-Host "먼저 DS폰에서 'Dashboard Drive' START 되어 있어야 함!"
Write-Host ""

python pc/ai/orchestrator.py --host $ROBOT_HOST --port 9999 `
    --real-planner --model $MODEL $percFlag --camera $CAMERA
