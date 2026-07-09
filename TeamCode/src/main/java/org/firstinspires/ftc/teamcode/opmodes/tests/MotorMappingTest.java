package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * 메카넘 모터 매핑/방향 진단 (연결 불필요, DS 화면만 봄).
 *
 * 목적: "직진은 되는데 옆이동/회전이 깨진다" → 모터 config 매핑 또는 배선이 틀렸는지 확정.
 *   모터를 하나씩 순서대로 천천히 정방향으로 돌린다. 화면에 지금 도는 모터 이름을 띄운다.
 *
 * 확인할 것 (각 모터가 돌 때):
 *   1. config 이름(lf/rf/lb/rb)과 실제 도는 물리 바퀴 위치가 맞나?
 *      (lf=왼쪽앞, rf=오른쪽앞, lb=왼쪽뒤, rb=오른쪽뒤)
 *   2. 정방향(+power) 명령 시 바퀴 윗면이 "앞으로" 구르나? (반대면 방향 반전 필요)
 *   3. 메카넘 롤러가 X자 배치인가? (앞에서 볼 때 좌우 바퀴 롤러가 V자로 마주봄)
 *
 * ⚠️ 안전: 바퀴 땅에서 살짝 띄우거나(권장) 여유공간. STOP 손 근처.
 */
@Autonomous(name = "Motor Mapping Test", group = "Test")
public class MotorMappingTest extends LinearOpMode {

    private static final double POWER = 0.25;     // 느리게
    private static final long EACH_MS = 2500;     // 각 모터 2.5초

    @Override
    public void runOpMode() {
        String[] names = {"lf", "rf", "lb", "rb"};
        String[] where = {"왼쪽-앞", "오른쪽-앞", "왼쪽-뒤", "오른쪽-뒤"};
        DcMotorEx[] motors = new DcMotorEx[4];
        for (int i = 0; i < 4; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, names[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addLine("모터 매핑 진단");
        telemetry.addLine("START 후 lf→rf→lb→rb 순으로 하나씩 2.5초 돎.");
        telemetry.addLine("각 모터가 돌 때: (1)그 이름의 물리 바퀴가 맞나 (2)윗면이 앞으로 구르나");
        telemetry.addLine("⚠️ 바퀴 살짝 띄우면 안전. STOP 손 근처");
        telemetry.update();

        waitForStart();

        for (int i = 0; i < 4 && opModeIsActive(); i++) {
            // 이 모터만 정방향
            for (int j = 0; j < 4; j++) motors[j].setPower(j == i ? POWER : 0.0);

            long t0 = System.currentTimeMillis();
            while (opModeIsActive() && System.currentTimeMillis() - t0 < EACH_MS) {
                telemetry.addLine(">>> 지금 도는 모터 <<<");
                telemetry.addData("config 이름", names[i]);
                telemetry.addData("여기여야 함(물리위치)", where[i]);
                telemetry.addData("확인", "이 바퀴가 맞나? 윗면이 앞으로 구르나?");
                telemetry.addData("순서", "%d / 4", i + 1);
                telemetry.update();
                sleep(50);
            }
            for (DcMotorEx m : motors) m.setPower(0);
            sleep(600); // 다음 모터 전 잠깐 멈춤
        }

        for (DcMotorEx m : motors) m.setPower(0);
        telemetry.addLine("끝. 각 모터의 (이름 vs 물리위치, 방향)을 기록하세요.");
        telemetry.update();
        sleep(2000);
    }
}
