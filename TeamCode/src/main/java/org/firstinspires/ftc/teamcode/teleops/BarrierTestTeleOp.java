package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.BoBot;

@Config
@TeleOp(name = "Barrier Test", group = "Test")
public class BarrierTestTeleOp extends LinearOpMode {

    public BoBot robot;
    private ElapsedTime loopTimer = new ElapsedTime();

    // Fine adjustment step size
    public static double POSITION_STEP = 0.05;

    // Button state tracking
    private boolean aButtonFirstInput = true;
    private boolean bButtonFirstInput = true;
    private boolean xButtonFirstInput = true;
    private boolean yButtonFirstInput = true;

    @Override
    public void runOpMode() {
        robot = new BoBot(this);

        telemetry.addData("Status", "Initialized - Ready to test barrier");
        telemetry.addData("", "");
        telemetry.addData("Controls:", "");
        telemetry.addData("A", "Set to OPEN position");
        telemetry.addData("B", "Set to CLOSED position");
        telemetry.addData("X", "Set to MIDDLE (0.5)");
        telemetry.addData("Y", "Reset to 0.0");
        telemetry.addData("D-Pad Up", "Increase position +0.05");
        telemetry.addData("D-Pad Down", "Decrease position -0.05");
        telemetry.addData("Right Bumper", "Fine tune +0.01");
        telemetry.addData("Left Bumper", "Fine tune -0.01");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            loopTimer.reset();

            // Control barrier with buttons
            controlBarrierPresets();
            controlBarrierManual();
            controlBarrierFine();

            // Display status
            displayTelemetry();

            telemetry.update();
        }
    }

    /**
     * Preset positions - A, B, X, Y buttons
     */
    private void controlBarrierPresets() {
        // A button - Open position
        if (gamepad1.a) {
            if (aButtonFirstInput) {
                aButtonFirstInput = false;
                robot.setBarrierOpen();
                telemetry.addData("Action", "Set to OPEN position");
            }
        } else {
            aButtonFirstInput = true;
        }

        // B button - Closed position
        if (gamepad1.b) {
            if (bButtonFirstInput) {
                bButtonFirstInput = false;
                robot.setBarrierClosed();
                telemetry.addData("Action", "Set to CLOSED position");
            }
        } else {
            bButtonFirstInput = true;
        }

        // X button - Middle position
        if (gamepad1.x) {
            if (xButtonFirstInput) {
                xButtonFirstInput = false;
                robot.barrierServo.setPosition(0.5);
                telemetry.addData("Action", "Set to MIDDLE (0.5)");
            }
        } else {
            xButtonFirstInput = true;
        }

        // Y button - Reset to 0
        if (gamepad1.y) {
            if (yButtonFirstInput) {
                yButtonFirstInput = false;
                robot.barrierServo.setPosition(0.0);
                telemetry.addData("Action", "RESET to 0.0");
            }
        } else {
            yButtonFirstInput = true;
        }
    }

    /**
     * Manual adjustment - D-Pad Up/Down
     */
    private void controlBarrierManual() {
        if (gamepad1.dpad_up) {
            double newPos = robot.barrierServo.getPosition() + POSITION_STEP;
            if (newPos <= 1.0) {
                robot.barrierServo.setPosition(newPos);
            }
            sleep(150); // Debounce
        }

        if (gamepad1.dpad_down) {
            double newPos = robot.barrierServo.getPosition() - POSITION_STEP;
            if (newPos >= 0.0) {
                robot.barrierServo.setPosition(newPos);
            }
            sleep(150); // Debounce
        }
    }

    /**
     * Fine adjustment - Bumpers
     */
    private void controlBarrierFine() {
        if (gamepad1.right_bumper) {
            double newPos = robot.barrierServo.getPosition() + 0.01;
            if (newPos <= 1.0) {
                robot.barrierServo.setPosition(newPos);
            }
            sleep(100); // Debounce
        }

        if (gamepad1.left_bumper) {
            double newPos = robot.barrierServo.getPosition() - 0.01;
            if (newPos >= 0.0) {
                robot.barrierServo.setPosition(newPos);
            }
            sleep(100); // Debounce
        }
    }

    private void displayTelemetry() {
        telemetry.addData("=== BARRIER STATUS ===", "");
        telemetry.addData("Current Position", String.format("%.3f", robot.barrierServo.getPosition()));
        telemetry.addData("Position (0-100%)", String.format("%.1f%%", robot.barrierServo.getPosition() * 100));
        telemetry.addData("", "");

        telemetry.addData("=== PRESET POSITIONS ===", "");
        telemetry.addData("BARRIER_OPEN_POS", String.format("%.3f", robot.BARRIER_OPEN_POS));
        telemetry.addData("BARRIER_CLOSED_POS", String.format("%.3f", robot.BARRIER_CLOSED_POS));
        telemetry.addData("", "");

        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("A", "Open Position");
        telemetry.addData("B", "Closed Position");
        telemetry.addData("X", "Middle (0.5)");
        telemetry.addData("Y", "Reset (0.0)");
        telemetry.addData("D-Pad Up/Down", String.format("Adjust ±%.2f", POSITION_STEP));
        telemetry.addData("Bumpers", "Fine ±0.01");
        telemetry.addData("", "");

        telemetry.addData("Loop Time (ms)", String.format("%.1f", loopTimer.time() * 1000.0));
        telemetry.addData("", "");
        telemetry.addData("TIP:", "Use D-Pad to find the right");
        telemetry.addData("", "open/closed positions, then");
        telemetry.addData("", "update constants in Dashboard!");
    }
}