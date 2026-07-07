package org.firstinspires.ftc.teamcode.opmodes.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BioBuzz TeleOp", group = "TeleOp")
public class BioBuzzTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        GamepadEx driver = new GamepadEx(gamepad1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            driver.readButtons();

            double leftX = driver.getLeftX();
            double leftY = driver.getLeftY();
            double rightX = driver.getRightX();

            telemetry.addData("Left X", leftX);
            telemetry.addData("Left Y", leftY);
            telemetry.addData("Right X", rightX);
            telemetry.update();
        }
    }
}
