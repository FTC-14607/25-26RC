package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.BoBot;


@Config
@TeleOp(name = "BoBot MainTeleOp", group = "Main")
public class MainTeleOp extends LinearOpMode {

    public BoBot robot;

    public static boolean showTelemetry = true;

    public static double RAMP_MAX_SPEED = 0.6;

    private ElapsedTime loopTimer = new ElapsedTime();


    private boolean intakeToggleFirstInput = true;
    private boolean intakeOn = false;


    public enum ShotMode { NEAR, FAR }

    private ShotMode shotMode = ShotMode.FAR;
    private double currentFlywheelPower = BoBot.FLYWHEEL_POWER_FAR;

    private boolean dpadUpFirstInput = true;
    private boolean dpadDownFirstInput = true;

    @Override
    public void runOpMode() {
        robot = new BoBot(this);
        robot.maxDrivePower = 0.9;

        initPositions();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            loopTimer.reset();

            robot.updateOrientation();

            //Gamepad1 stuff it controls, subject to change
            controlDriveTrain(gamepad1);
            controlShotMode(gamepad1);
            controlRampAngle(gamepad1);

            // Gamepad2 stuff it controls, subject to change
            controlIntake(gamepad2);
            controlShooting(gamepad2);
            controlMaxSpeedDriveTrain(gamepad2);

            if (showTelemetry) {
                telemetry.addData("Shot Mode", shotMode);
                telemetry.addData("Flywheel Power", currentFlywheelPower);
                telemetry.addData("Intake On", intakeOn);
                telemetry.addData("Barrier Pos", robot.barrierServo.getPosition());
                telemetry.addData("Ramp Pos", robot.rampServo.getPosition());
                telemetry.addData("Max Drive Power", robot.maxDrivePower);
                telemetry.addData("Loop Speed (ms)", loopTimer.time() * 1000.0);
                telemetry.update();
            }
        }
    }

    private void initPositions() {
       //intital positions, we assume its far
        shotMode = ShotMode.FAR;
        currentFlywheelPower = BoBot.FLYWHEEL_POWER_FAR;

        robot.setBarrierClosed();
        robot.setRampFar();
        robot.stopFlywheel();
        robot.stopIntake();
    }



    public void controlDriveTrain(Gamepad gamepad) {
        double throttle = gamepad.left_stick_y;
        double strafe   = -gamepad.left_stick_x;
        double rotate   = gamepad.right_stick_x;

        robot.drive(throttle, strafe, rotate);
    }


    private void controlShotMode(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            if (dpadUpFirstInput) {
                dpadUpFirstInput = false;
                shotMode = ShotMode.FAR;
                currentFlywheelPower = BoBot.FLYWHEEL_POWER_FAR;
                robot.setRampFar();
            }
        } else {
            dpadUpFirstInput = true;
        }

        if (gamepad.dpad_down) {
            if (dpadDownFirstInput) {
                dpadDownFirstInput = false;
                shotMode = ShotMode.NEAR;
                currentFlywheelPower = BoBot.FLYWHEEL_POWER_NEAR;
                robot.setRampNear();
            }
        } else {
            dpadDownFirstInput = true;
        }
    }


    private void controlIntake(Gamepad gamepad) {
        boolean input = gamepad.x;

        if (input) {
            if (intakeToggleFirstInput) {
                intakeToggleFirstInput = false;
                intakeOn = !intakeOn;

                if (intakeOn) {
                    robot.setIntakePower(BoBot.INTAKE_POWER);
                } else {
                    robot.stopIntake();
                }
            }
        } else {
            intakeToggleFirstInput = true;
        }
    }


    private void controlShooting(Gamepad gamepad) {
        boolean bothTriggers = (gamepad.left_trigger > 0.5 && gamepad.right_trigger > 0.5);

        if (bothTriggers) {
            robot.setBarrierOpen();
            robot.setFlywheelPower(currentFlywheelPower);
            if (intakeOn) {
                robot.setIntakePower(BoBot.INTAKE_FEED_POWER);
            }
        } else {
            robot.setBarrierClosed();
            robot.stopFlywheel();
            if (intakeOn) {
                robot.setIntakePower(BoBot.INTAKE_POWER);
            }
        }
    }

    private void controlRampAngle(Gamepad gamepad) {
        double input = 0.0;

        if (gamepad.dpad_right) {
            input = 1.0;
        } else if (gamepad.dpad_left) {
            input = -1.0;
        }

        if (input != 0.0) {
            double deltaTime = loopTimer.time();

            double nextRampPos = robot.getRampPos() + input * deltaTime * RAMP_MAX_SPEED;
            robot.setRampPos(nextRampPos);
        }
    }
    public void controlMaxSpeedDriveTrain(Gamepad gamepad) {
        if      (gamepad.dpad_down)  robot.maxDrivePower = 0.3;
        else if (gamepad.dpad_left)  robot.maxDrivePower = 0.5;
        else if (gamepad.dpad_up)    robot.maxDrivePower = 0.7;
        else if (gamepad.dpad_right) robot.maxDrivePower = 0.9;
    }
}
