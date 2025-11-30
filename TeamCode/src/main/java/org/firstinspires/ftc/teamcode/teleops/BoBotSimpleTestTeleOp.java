package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.BoBotSimple;

@TeleOp(name = "BoBot Motor Test", group = "Test")
public class BoBotSimpleTestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        BoBotSimple robot = new BoBotSimple(this);

        telemetry.addLine("BoBotSimple Motor Test Ready");
        telemetry.addLine(" - Left Stick Y = Intake motors");
        telemetry.addLine(" - Right Trigger = Flywheel forward");
        telemetry.addLine(" - Left Trigger = Flywheel reverse");
        telemetry.addLine(" - A Button = Stop ALL motors");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Intake test (left stick Y) ---
            double intakePower = -gamepad1.left_stick_y; // up = positive
            robot.setIntakePower(intakePower);

            // --- Flywheel test (triggers) ---
            double flywheelPower = gamepad1.right_trigger - gamepad1.left_trigger;
            robot.setFlywheelPower(flywheelPower);

            // --- Emergency stop ---
            if (gamepad1.a) {
                robot.stopFlywheel();
                robot.stopIntake();
            }

            // Telemetry
            telemetry.addData("Intake Power", intakePower);
            telemetry.addData("Flywheel Power", flywheelPower);
            telemetry.update();

            idle();
        }
    }
}
