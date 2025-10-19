package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Simple Max Motor Test", group = "Test")
public class SimpleMaxMotorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Initialize motor in slot 1 (typically "motor0")
        DcMotor motor = hardwareMap.get(DcMotor.class, "motor0");

        telemetry.addLine("Ready to run");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            motor.setPower(-1.0); // Max power (forward)
            telemetry.addLine("Motor running at full power");
            telemetry.update();

            // Keep motor running until stop
            while (opModeIsActive()) {
                idle();
            }

            motor.setPower(0); // Stop motor when opmode ends
        }
    }
}