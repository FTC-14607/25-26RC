package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Hood Servo Test", group = "Test")
public class HoodServoTest extends LinearOpMode {

    private Servo hoodServo;
    private double currentPosition = 0.5;

    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private boolean aPressed = false;
    private boolean bPressed = false;
    private boolean xPressed = false;
    private boolean yPressed = false;

    @Override
    public void runOpMode() {
        // Initialize hardware
        hoodServo = hardwareMap.get(Servo.class, "rampServo");

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Press START to begin test");
        telemetry.update();

        waitForStart();

        // Set to middle position initially
        hoodServo.setPosition(0.5);
        currentPosition = 0.5;

        telemetry.addData("Status", "Running");
        telemetry.addLine();
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("A = Set to 0.0");
        telemetry.addLine("B = Set to 1.0");
        telemetry.addLine("X = Set to 0.5 (middle)");
        telemetry.addLine("Y = Toggle position test");
        telemetry.addLine("D-Pad Up = Increase by 0.05");
        telemetry.addLine("D-Pad Down = Decrease by 0.05");
        telemetry.addLine();
        telemetry.addLine("WHAT TO LOOK FOR:");
        telemetry.addLine("POSITIONAL: Servo moves to angle and stops");
        telemetry.addLine("CONTINUOUS: 0.5=stop, 0.0/1.0=spin");
        telemetry.update();

        sleep(3000);

        while (opModeIsActive()) {
            // Button A - Set to 0.0
            if (gamepad1.a && !aPressed) {
                aPressed = true;
                currentPosition = 0.0;
                hoodServo.setPosition(currentPosition);
            } else if (!gamepad1.a) {
                aPressed = false;
            }

            // Button B - Set to 1.0
            if (gamepad1.b && !bPressed) {
                bPressed = true;
                currentPosition = 1.0;
                hoodServo.setPosition(currentPosition);
            } else if (!gamepad1.b) {
                bPressed = false;
            }

            // Button X - Set to 0.5 (middle/stop)
            if (gamepad1.x && !xPressed) {
                xPressed = true;
                currentPosition = 0.5;
                hoodServo.setPosition(currentPosition);
            } else if (!gamepad1.x) {
                xPressed = false;
            }

            // Button Y - Run position test sequence
            if (gamepad1.y && !yPressed) {
                yPressed = true;
                runPositionTest();
            } else if (!gamepad1.y) {
                yPressed = false;
            }

            // D-Pad Up - Increase position
            if (gamepad1.dpad_up && !dpadUpPressed) {
                dpadUpPressed = true;
                currentPosition += 0.05;
                if (currentPosition > 1.0) currentPosition = 1.0;
                hoodServo.setPosition(currentPosition);
            } else if (!gamepad1.dpad_up) {
                dpadUpPressed = false;
            }

            // D-Pad Down - Decrease position
            if (gamepad1.dpad_down && !dpadDownPressed) {
                dpadDownPressed = true;
                currentPosition -= 0.05;
                if (currentPosition < 0.0) currentPosition = 0.0;
                hoodServo.setPosition(currentPosition);
            } else if (!gamepad1.dpad_down) {
                dpadDownPressed = false;
            }

            // Display telemetry
            telemetry.addData("Current Position", "%.2f", currentPosition);
            telemetry.addData("Servo Position", "%.2f", hoodServo.getPosition());
            telemetry.addLine();
            telemetry.addLine("OBSERVE THE SERVO:");
            telemetry.addLine("- Does it move to angles and HOLD?");
            telemetry.addLine("- Or does 0.0/1.0 make it SPIN continuously?");
            telemetry.addLine("- Does 0.5 make it STOP spinning?");
            telemetry.addLine();
            telemetry.addData("Diagnosis", getDiagnosis());
            telemetry.update();
        }
    }

    private void runPositionTest() {
        telemetry.addData("Test", "Running position sequence...");
        telemetry.update();

        // Test 0.0
        hoodServo.setPosition(0.0);
        currentPosition = 0.0;
        sleep(2000);

        // Test 0.5
        hoodServo.setPosition(0.5);
        currentPosition = 0.5;
        sleep(2000);

        // Test 1.0
        hoodServo.setPosition(1.0);
        currentPosition = 1.0;
        sleep(2000);

        // Back to 0.5
        hoodServo.setPosition(0.5);
        currentPosition = 0.5;

        telemetry.addData("Test", "Sequence complete!");
        telemetry.update();
    }

    private String getDiagnosis() {
        if (currentPosition == 0.5) {
            return "If servo is STOPPED -> CONTINUOUS\nIf at middle angle -> POSITIONAL";
        } else if (currentPosition == 0.0 || currentPosition == 1.0) {
            return "If servo is SPINNING -> CONTINUOUS\nIf at min/max angle -> POSITIONAL";
        } else {
            return "Observe servo behavior";
        }
    }
}