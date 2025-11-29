package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Config
public class BoBot extends MecanumDrive {

    //hardware start region

    //bottom intake motor
    public DcMotorEx intakeLower;

    //upper intake motor
    public DcMotorEx intakeUpper;

    //flywheel motor at the top
    public DcMotorEx flywheel;

    // Binary servo purpose (1: yes it goes up ramp, 0: blocks it)
    public Servo barrierServo;

    //changing this causing a change in ramp angle
    public Servo rampServo;

    //endregion

    //TODO: PLEASE TUNE THESE FUCKING CONSTANTS ---- ASAP IT SHJOULDN'T BE THAT HARD

    //this is from floor(lower one), change based on time it takes as well as slippage
    public static double INTAKE_POWER = 0.8;

    //power used for intake feeding change based on time we have
    public static double INTAKE_FEED_POWER = 0.8;

    //flywheel power for a near shot
    public static double FLYWHEEL_POWER_NEAR = 0.6;

    //flywheel power for a far shot
    public static double FLYWHEEL_POWER_FAR = 0.9;

    //barrier servo positioning for closed(not allowing balls to go through)
    public static double BARRIER_CLOSED = 0.0;

    //barrier servo positioning for open
    public static double BARRIER_OPEN = 0.5;

    //Ramp servo positioning for a near shot pls tune yall
    public static double RAMP_NEAR_POS = 0.35;

    //Ramp servo positioning for a far shot pls tune yall
    public static double RAMP_FAR_POS = 0.65;

    //Min and max range for the ramp, subject to change based on the tread width
    public static double RAMP_MIN_POS = 0.20;
    public static double RAMP_MAX_POS = 0.80;


    public boolean showTelemetry = true;

    //endregion

    public BoBot(LinearOpMode opmode) {
        super(opmode);


        ROBOT_HEIGHT = 18;
        ROBOT_LENGTH = 18;
        ROBOT_WIDTH  = 18;

        // 96mm goBILDA mecanum wheels  -- we use the same each year,
        WHEEL_DIAMETER      = 9.6 / 2.54;
        WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        DRIVETRAIN_TICKS    = 537.68984;

        // Tuning for strafing
        STRAFE_MULTIPLIER           = 1.3;
        STRAIGHT_ROTATION_CORRECTION = 0;
        STRAFE_ROTATION_CORRECTION   = 0;

        // Hardware mapping is right here, for programmers make sure they match up

        intakeLower  = hardwareMap.get(DcMotorEx.class, "intakeLower");
        intakeUpper  = hardwareMap.get(DcMotorEx.class, "intakeUpper");
        flywheel     = hardwareMap.get(DcMotorEx.class, "flywheel");
        barrierServo = hardwareMap.get(Servo.class, "barrierServo");
        rampServo    = hardwareMap.get(Servo.class, "rampServo");


        intakeLower.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeUpper.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);


        intakeLower.setDirection(DcMotorEx.Direction.FORWARD);
        intakeUpper.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel.setDirection(DcMotorEx.Direction.FORWARD);


        imu.resetYaw();
        updateOrientation();

        // default positioning for all these
        setBarrierClosed();
        setRampFar();
        stopFlywheel();
        stopIntake();
    }


    public void updateOrientation() {
        orientation = imu.getRobotYawPitchRollAngles();
    }

    public void setIntakePower(double power) {
        intakeLower.setPower(power);
        intakeUpper.setPower(power);
    }

    public void stopIntake() {
        setIntakePower(0.0);
    }

    public void setFlywheelPower(double power) {
        flywheel.setPower(power);
    }

    public void stopFlywheel() {
        flywheel.setPower(0.0);
    }

    public void setBarrierClosed() {
        barrierServo.setPosition(BARRIER_CLOSED);
    }

    public void setBarrierOpen() {
        barrierServo.setPosition(BARRIER_OPEN);
    }

    public double getRampPos() {
        return rampServo.getPosition();
    }

    public void setRampPos(double pos) {
        rampServo.setPosition(clip(pos, RAMP_MIN_POS, RAMP_MAX_POS));
    }

    public void setRampNear() {
        setRampPos(RAMP_NEAR_POS);
    }

    public void setRampFar() {
        setRampPos(RAMP_FAR_POS);
    }


    //endregion
}
