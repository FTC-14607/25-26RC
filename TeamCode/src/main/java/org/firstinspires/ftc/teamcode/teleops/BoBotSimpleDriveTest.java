package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.BoBotSimple;

@TeleOp(name="BoBotSimple Drive Train Test", group="Test")
public class BoBotSimpleDriveTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        BoBotSimple robot = new BoBotSimple(this);

        telemetry.addLine("BoBotSimple Drive Test Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ---------- Mecanum Drive ----------
            double throttle = -gamepad1.left_stick_y;   // forward/back
            double strafe   =  gamepad1.left_stick_x;   // left/right
            double rotate   =  gamepad1.right_stick_x;  // turn

            robot.drive(throttle, strafe, rotate);

            // ---------- Telemetry ----------
            telemetry.addLine("=== Drive Train ===");
            telemetry.addData("Throttle", throttle);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Rotate", rotate);

            telemetry.addData("Front Left Encoder", robot.frontLeft.getCurrentPosition());
            telemetry.addData("Front Right Encoder", robot.frontRight.getCurrentPosition());
            telemetry.addData("Back Left Encoder", robot.backLeft.getCurrentPosition());
            telemetry.addData("Back Right Encoder", robot.backRight.getCurrentPosition());

            telemetry.update();
        }
    }
}
