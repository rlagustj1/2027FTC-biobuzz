package org.firstinspires.ftc.teamcode.opmodes.tests;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;

/**
 * 로봇 카메라(Control Hub 연결) 색검출 테스트 — 자율주행 "인식" 계층의 1단계.
 * VisionPortal + ColorBlobLocatorProcessor로 빨강(목표)·파랑(장애물) 블록을 검출한다.
 *
 * ⚙️ 설정 필요: DS폰 로봇 config에 웹캠을 "Webcam 1" 이름으로 추가 (Control Hub USB에 연결).
 *   카메라 미연결이면 INIT에서 실패하므로, 카메라 달고 config 등록 후 실행할 것.
 *
 * 동작: INIT부터 카메라 스트림 시작(DS에서 미리보기 가능). 화면(텔레메트리)에 검출된
 *   빨강/파랑 블록의 개수 + 화면 위치(픽셀) + 대략 방향(좌/중/우, 가까움/멈)을 표시.
 *
 * 다음 단계: ① 픽셀→필드좌표 보정 ② 검출을 PC로 전송(통신) ③ 회피 로직(반사층) ④ AI 판단.
 * 이 단계는 "카메라가 색을 제대로 보는지"만 확인한다. (AI/Ollama 불필요)
 *
 * ⚠️ 조명/실제 블록 색에 따라 검출이 다를 수 있음. 안 잡히면 해상도·색범위 조정.
 */
@TeleOp(name = "Vision Color Test", group = "Test")
public class VisionColorTest extends LinearOpMode {

    // 검출할 최소 blob 크기(픽셀 면적). 노이즈 걸러냄.
    private static final int MIN_AREA = 100;
    private static final int MAX_AREA = 30000;

    @Override
    public void runOpMode() {
        // 빨강(목표), 파랑(장애물) 각각의 색검출기
        ColorBlobLocatorProcessor redLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        ColorBlobLocatorProcessor blueLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        VisionPortal portal;
        try {
            portal = new VisionPortal.Builder()
                    .addProcessor(redLocator)
                    .addProcessor(blueLocator)
                    .setCameraResolution(new Size(320, 240))
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .build();
        } catch (Exception e) {
            telemetry.addLine("카메라 초기화 실패 — config에 'Webcam 1' 등록됐나 확인");
            telemetry.addData("에러", e.getMessage());
            telemetry.update();
            waitForStart();
            return;
        }

        telemetry.setMsTransmissionInterval(100);
        telemetry.addLine("카메라 스트림 시작. DS에서 미리보기 확인.");
        telemetry.addLine("빨강=목표, 파랑=장애물. START 안 눌러도 INIT에서 검출됨.");
        telemetry.update();

        while (opModeIsActive() || opModeInInit()) {
            reportBlobs("빨강(목표)", redLocator);
            reportBlobs("파랑(장애물)", blueLocator);
            telemetry.update();
            sleep(80);
        }

        portal.close();
    }

    /** 한 색의 blob들을 필터링해 개수·위치·방향을 텔레메트리로 출력. */
    private void reportBlobs(String label, ColorBlobLocatorProcessor locator) {
        List<ColorBlobLocatorProcessor.Blob> blobs = locator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, MIN_AREA, MAX_AREA, blobs);

        telemetry.addData("── " + label, "개수 %d", blobs.size());
        int n = 0;
        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            if (n++ >= 3) break;  // 상위 3개만 (면적 큰 순)
            RotatedRect box = b.getBoxFit();
            double cx = box.center.x;   // 화면 가로 픽셀 (0=왼, 320=오른)
            double cy = box.center.y;   // 화면 세로 픽셀 (0=위, 240=아래)
            // 대략 방향: 화면 가로 위치로 좌/중/우, 세로(아래일수록 가까움)로 거리감
            String side = cx < 107 ? "왼쪽" : (cx > 213 ? "오른쪽" : "중앙");
            String dist = cy > 160 ? "가까움" : (cy < 80 ? "멈" : "중간");
            telemetry.addLine(String.format("   (%3.0f,%3.0f) 면적%5d  %s·%s",
                    cx, cy, (int) b.getContourArea(), side, dist));
        }
    }
}
