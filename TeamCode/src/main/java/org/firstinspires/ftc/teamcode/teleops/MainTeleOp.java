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

    //region teleop constants
    public static double RAMP_MAX_SPEED = 0.6; // virtual units / sec
    public static boolean showTelemetry = true;
    //endregion

    //region state
    public BoBot robot;

    public ElapsedTime delayTimer = new ElapsedTime();
    public ElapsedTime loopTimer = new ElapsedTime();
    public ElapsedTime timerOffset = new ElapsedTime();

    private boolean intakeToggleFirstInput = true;
    private boolean intakeOn = false;
    private boolean shootingOn = false;

    public enum ShotMode { NEAR, FAR }
    private ShotMode shotMode = ShotMode.FAR;

    private double currentFlywheelVelocity = BoBot.TARGET_VELO_FAR;

    //given lookup table to change our flywheel velocity to
    private double [] flywheelVelocityLookup  = {600,1200,1600,2000};



    private boolean dpadUpFirstInput = true;
    private boolean dpadDownFirstInput = true;
    //endregion
    @Override
    public void runOpMode() {
        robot = new BoBot(this);
        robot.maxDrivePower = 0.9;
        initPositions();
        loopTimer.reset();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();

            robot.update();


            //Gamepad1 stuff it controls, subject to change
            controlDriveTrain(gamepad1);
            //controlShotMode(gamepad1);
            controlMaxSpeedDriveTrain(gamepad1);
            controlRampAngle(gamepad2, dt);

            // Gamepad2 stuff it controls, subject to change

            controlIntake(gamepad2);
            controlShooting(gamepad2);
            //controlBumperServo(gamepad2);
            robot.updateRamp();



            if (showTelemetry) {
                telemetry.addData("Shot Mode", shotMode);
                telemetry.addData("Flywheel Target Velo", currentFlywheelVelocity);
                telemetry.addData("Flywheel Velo", "%.1f", robot.getFlywheelVelocity());
                telemetry.addData("Intake On", intakeOn);

                telemetry.addData("Ramp Target", "%.3f", robot.getRampPos());
                telemetry.addData("Ramp Est", "%.3f", robot.getRampPosEstimate());
                telemetry.addData("Ramp FAR preset", "%.3f", BoBot.RAMP_FAR_POS);
                telemetry.addData("Ramp NEAR preset", "%.3f", BoBot.RAMP_NEAR_POS);

                telemetry.addData("Barrier Position", robot.barrierServo.getPosition());
                telemetry.addData("Max Drive Power", robot.maxDrivePower);
                telemetry.addData("Loop Speed", "%5.2f ms", loopTimer.time() * 1000.0);
                telemetry.update();
            }
        }
    }

    //region init
    public void initPositions() {
        // initial positions, assume FAR
        shotMode = ShotMode.FAR;
        currentFlywheelVelocity = BoBot.TARGET_VELO_FAR;

        robot.setBarrierClosed();
        robot.setRampFar();
        robot.stopFlywheel();
        robot.stopIntake();
    }
    //endregion
    public void controlDriveTrain(Gamepad gamepad) {
        double throttle = gamepad.left_stick_y;
        double strafe   = -gamepad.left_stick_x;
        double rotate   = gamepad.right_stick_x;

        robot.drive(throttle, strafe, rotate);
    }
    public void controlShotMode(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            if (dpadUpFirstInput) {
                dpadUpFirstInput = false;
                shotMode = ShotMode.FAR;
                currentFlywheelVelocity = BoBot.TARGET_VELO_FAR;
                robot.setRampFar();
            }
        } else {
            dpadUpFirstInput = true;
        }

        if (gamepad.dpad_down) {
            if (dpadDownFirstInput) {
                dpadDownFirstInput = false;
                shotMode = ShotMode.NEAR;
                currentFlywheelVelocity = BoBot.TARGET_VELO_NEAR;
                robot.setRampNear();
            }
        } else {
            dpadDownFirstInput = true;
        }
    }
    /*
    public void controlShotPower(Gamepad gamepad){
        int indexCurr = 0;
        if (gamepad.dpad_up){

        }
        else if(gamepad.dpad_down){
            indexCurr -= 1;

        }

    }
    */

    public void controlRampAngle(Gamepad gamepad, double dt) {
        double input = 0.0;
        if (gamepad.dpad_up) input = 1.0;
        else if (gamepad.dpad_down) input = -1.0;
        if (input != 0.0) {
            double nextRampPos = robot.getRampPos() + input * dt * RAMP_MAX_SPEED;
            robot.setRampPos(nextRampPos);
            if (shotMode == ShotMode.FAR) {
                BoBot.RAMP_FAR_POS = robot.getRampPos();
            }
            else {
                BoBot.RAMP_NEAR_POS = robot.getRampPos();
            }
        }
    }

    private void controlIntake(Gamepad gamepad) {
        boolean posInput = gamepad.x;
        boolean negInput = gamepad.b;


        if (posInput) {
            if (intakeToggleFirstInput) {
                intakeToggleFirstInput = false;
                intakeOn = !intakeOn;

                if (intakeOn) {
                    robot.setIntakePower(BoBot.INTAKE_POWER);
                } else {
                    robot.stopIntake();
                }
            }
        } else if(negInput){
            if (intakeToggleFirstInput) {
                intakeToggleFirstInput = false;
                intakeOn = !intakeOn;

                if (intakeOn) {
                    robot.setIntakePower(BoBot.INTAKE_POWER * -1);
                } else {
                    robot.stopIntake();
                }
            }
        }
        else {
            intakeToggleFirstInput = true;
        }
    }


    private void controlShooting(Gamepad gamepad) {
        boolean bothTriggers = (gamepad.left_trigger > 0.5 && gamepad.right_trigger > 0.5);

        if (bothTriggers) {
            // Turn flywheel on
            shootingOn = true;
            robot.setFlywheelVelocity(currentFlywheelVelocity);


        } else {
            // Turn flywheel off
            shootingOn = false;
            timerOffset.reset();

            robot.stopFlywheel();
        }
    }
    private void controlBumperServo(Gamepad gamepad){
        boolean input = gamepad.y;
        if (input){
            robot.setBarrierClosed();
            if (delayTimer.seconds() > 1.2)
                robot.setBarrierOpen();

        }
        else{
            robot.setBarrierOpen();
            delayTimer.reset();
        }

    }
    public void controlMaxSpeedDriveTrain(Gamepad gamepad)
    {
        boolean leftTriggerHeld = gamepad.left_trigger>0.5;
        // change max drive power
        if(leftTriggerHeld)
            robot.maxDrivePower = 0.3;
        else
            robot.maxDrivePower = 0.9;
    }

}