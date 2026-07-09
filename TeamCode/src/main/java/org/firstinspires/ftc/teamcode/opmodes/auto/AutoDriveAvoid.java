package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.io.File;
import java.io.FileWriter;
import java.util.List;

/**
 * 목표 이동 + 카메라 기반 장애물 회피 (반사층). 자율주행 다중 프로세싱의 "빠른 반사" 계층.
 *
 * 원리(포텐셜 필드): 목표는 로봇을 당기고(goto P제어), 파란 장애물은 밀어낸다(카메라).
 *   두 힘을 합쳐 로봇 로컬 속도로 준다 → 목표로 가되 장애물은 옆으로 비켜 감.
 *   장애물 회피는 카메라 픽셀 위치로 즉시 반응(로봇 로컬) — PC/AI 없이 빠르게.
 *
 * 목표: 테스트용 고정(전방 TARGET). 앞에 파란 물체 두면 피해서 가는지 확인.
 *   (실사용 시 목표는 대시보드/AI가 goto로 줌 — 이 회피 로직을 그대로 얹으면 됨)
 *
 * ⚙️ 웹캠 config 이름 "Webcam 1". SIGN=+1(드라이브 수정 후). Localizer 오프셋 실측 반영됨.
 * ⚠️ 안전: 넓은 공간, 저속, STOP 손 근처.
 */
@Autonomous(name = "Auto Drive + Avoid", group = "auto")
public class AutoDriveAvoid extends LinearOpMode {

    // 테스트 목표 (전방 1.2m). 실사용 시 대시보드 goto로 대체.
    private static final double TARGET_X = 1.2, TARGET_Y = 0.0, TARGET_HEADING = 0.0;

    // 제어 게인 (GoToPointTest와 정렬, 저속)
    private static final double SIGN = +1.0;
    private static final double KP_POS = 0.9, MAX_DRIVE = 0.25;
    private static final double KP_HEADING = 0.6, MAX_TURN = 0.35, MIN_TURN_POWER = 0.15;
    private static final double POS_TOLERANCE = 0.05;
    private static final double HEADING_TOLERANCE = Math.toRadians(8);
    private static final double RESUME_THRESHOLD = Math.toRadians(14);
    private static final double MOVE_HEADING_GATE = Math.toRadians(25);

    // 회피 파라미터 (카메라 320x240 기준)
    private static final int FRAME_W = 320, FRAME_H = 240;
    private static final int OBST_MIN_AREA = 250;    // 낮춤: 더 멀리(작을 때)부터 반응 → 늦음 개선
    private static final double K_AVOID = 0.55;      // 높임: 옆으로 더 세게 비킴
    private static final double AREA_FULL = 4000.0;  // 낮춤: 더 일찍 최대회피 도달

    @Override
    public void runOpMode() {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        Localizer loc = new Localizer(hardwareMap, "perpEncoder", "parallelEncoder");
        drive.setOpenLoop();

        // 파란 장애물 검출기 + 비전포털
        ColorBlobLocatorProcessor blue = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setBlurSize(5)
                .build();
        VisionPortal portal;
        try {
            portal = new VisionPortal.Builder()
                    .addProcessor(blue)
                    .setCameraResolution(new Size(FRAME_W, FRAME_H))
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .build();
        } catch (Exception e) {
            telemetry.addLine("카메라 실패 — config 'Webcam 1' 확인"); telemetry.update();
            waitForStart(); return;
        }

        telemetry.addLine("목표 전방 1.2m. 앞에 파란 물체 두면 피해서 감.");
        telemetry.addLine("START. 넓은 공간, STOP 손 근처.");
        telemetry.update();
        waitForStart();
        loc.resetPose();

        boolean headingLatched = false;

        // 주행 로그 (연결 불필요). 끝나면 adb pull /sdcard/avoidlog.csv
        FileWriter fw = null;
        try {
            fw = new FileWriter(new File("/sdcard/avoidlog.csv"));
            fw.write("t_ms,x,y,h_deg,posErr,obstCount,maxStrength,avoidVy,vx,vy,w,blobCx,blobArea\n");
        } catch (Exception ignored) {}
        long t0 = System.currentTimeMillis();

        while (opModeIsActive()) {
            loc.update();
            double x = loc.getX(), y = loc.getY(), th = loc.getHeading();

            // --- 목표 인력: goto P제어 (로컬프레임) ---
            double exField = TARGET_X - x, eyField = TARGET_Y - y;
            double c = Math.cos(-th), s = Math.sin(-th);
            double exLocal = exField * c - eyField * s;
            double eyLocal = exField * s + eyField * c;
            double eHeading = normalizeAngle(TARGET_HEADING - th);
            double posErr = Math.hypot(exField, eyField);
            double absHeadingErr = Math.abs(eHeading);

            double vx = 0, vy = 0, w = 0;
            boolean aligned = absHeadingErr < MOVE_HEADING_GATE;
            if (posErr >= POS_TOLERANCE && aligned) {
                vx = clamp(KP_POS * exLocal, -MAX_DRIVE, MAX_DRIVE);
                vy = clamp(KP_POS * eyLocal, -MAX_DRIVE, MAX_DRIVE);
            }
            if (headingLatched && absHeadingErr > RESUME_THRESHOLD) headingLatched = false;
            else if (!headingLatched && absHeadingErr < HEADING_TOLERANCE) headingLatched = true;
            if (!headingLatched) {
                w = clamp(SIGN * KP_HEADING * eHeading, -MAX_TURN, MAX_TURN);
                if (absHeadingErr > RESUME_THRESHOLD && Math.abs(w) < MIN_TURN_POWER)
                    w = Math.copySign(MIN_TURN_POWER, w);
            }

            // --- 장애물 척력: 카메라 파란 blob → 옆으로 밀기 (반사) ---
            double avoidVy = 0;
            double vxScale = 1.0;
            double maxStrength = 0;   // 가장 가까운 장애물의 세기 (진동 억제용)
            int obstCount = 0;
            double logBlobCx = -1, logBlobArea = 0;   // 로깅용: 가장 큰 blob
            List<ColorBlobLocatorProcessor.Blob> blobs = blue.getBlobs();
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, OBST_MIN_AREA, 100000, blobs);
            for (ColorBlobLocatorProcessor.Blob b : blobs) {
                RotatedRect box = b.getBoxFit();
                double cx = box.center.x;                 // 0=왼, FRAME_W=오른
                double area = b.getContourArea();
                double strength = Math.min(1.0, area / AREA_FULL);  // 가까울수록↑
                // 장애물이 왼쪽이면 오른쪽으로(=vy 음수), 오른쪽이면 왼쪽으로(=vy 양수) 비킴
                double dir = (cx < FRAME_W / 2.0) ? -1.0 : +1.0;
                avoidVy += dir * strength * K_AVOID;
                vxScale = Math.min(vxScale, 1.0 - strength * 0.85);  // 가까우면 전진 더 감속
                maxStrength = Math.max(maxStrength, strength);
                obstCount++;
                if (area > logBlobArea) { logBlobArea = area; logBlobCx = cx; }
            }

            // --- 인력 + 척력 합성 ---
            // ★진동(헤맴) 억제: 장애물이 가까이 보이는 동안은 목표의 '옆쪽 당김(vy)'을 줄인다.
            //   안 그러면 피했다가 목표가 다시 장애물 쪽으로 당겨 오락가락함. 회피가 이기게 해서
            //   한쪽으로 '돌아가게' 함. (전진 vx는 유지 → 계속 앞으로는 감)
            vy *= (1.0 - maxStrength);
            vx *= vxScale;
            vy = clamp(vy + avoidVy, -(MAX_DRIVE + K_AVOID), MAX_DRIVE + K_AVOID);

            boolean reached = posErr < POS_TOLERANCE && absHeadingErr < HEADING_TOLERANCE;
            drive.driveRobotRelative(reached ? 0 : vx, reached ? 0 : vy, reached ? 0 : w);

            if (fw != null) {
                try {
                    fw.write(String.format("%d,%.3f,%.3f,%.1f,%.3f,%d,%.2f,%.3f,%.3f,%.3f,%.3f,%.0f,%.0f\n",
                            System.currentTimeMillis() - t0, x, y, Math.toDegrees(th), posErr,
                            obstCount, maxStrength, avoidVy, vx, vy, w, logBlobCx, logBlobArea));
                } catch (Exception ignored) {}
            }

            telemetry.addData("상태", reached ? ">>> 도착!" : "이동중");
            telemetry.addData("pose", "X=%.2f Y=%.2f θ=%.0f°", x, y, Math.toDegrees(th));
            telemetry.addData("남은거리", "%.2f m", posErr);
            telemetry.addData("장애물(파랑)", "%d개  회피vy=%.2f  전진x%.2f", obstCount, avoidVy, vxScale);
            telemetry.addData("출력", "vx=%.2f vy=%.2f w=%.2f", vx, vy, w);
            telemetry.update();
        }
        drive.driveRobotRelative(0, 0, 0);
        if (fw != null) { try { fw.flush(); fw.close(); } catch (Exception ignored) {} }
        portal.close();
    }

    private static double normalizeAngle(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
