package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class BoBotSimple extends MecanumDrive {

    // --- Motor Hardware Only ---

    public DcMotorEx intakeLower;
    public DcMotorEx flywheel;
    public CRServo bumperServo;

    // Adjustable motor constants for testing
    public static double INTAKE_POWER = 0.8;
    public static double INTAKE_FEED_POWER = 0.8;

    public static double FLYWHEEL_POWER_NEAR = 0.6;
    public static double FLYWHEEL_POWER_FAR = 1.0;

    public static double BARRIER_CLOSED = 0.0;

    //barrier servo positioning for open
    public static double BARRIER_OPEN = 0.5;

    public boolean showTelemetry = true;

    public BoBotSimple(LinearOpMode opmode) {
        super(opmode);

        // Robot geometry
        ROBOT_HEIGHT = 18;
        ROBOT_LENGTH = 18;
        ROBOT_WIDTH  = 18;

        // Drivetrain config
        WHEEL_DIAMETER      = 9.6 / 2.54;
        WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        DRIVETRAIN_TICKS    = 537.68984;

        STRAFE_MULTIPLIER            = 1.3;
        STRAIGHT_ROTATION_CORRECTION = 0;
        STRAFE_ROTATION_CORRECTION   = 0;
        bumperServo = hardwareMap.get(CRServo.class, "bumperServo");
        // Map motors
        intakeLower  = hardwareMap.get(DcMotorEx.class, "intakeLower");
        flywheel     = hardwareMap.get(DcMotorEx.class, "flywheel");

        // Zero power behavior
        intakeLower.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Direction
        intakeLower.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel.setDirection(DcMotorEx.Direction.FORWARD);

        imu.resetYaw();
        updateOrientation();

        // Default motor states
        stopFlywheel();
        stopIntake();
    }

    public void updateOrientation() {
        orientation = imu.getRobotYawPitchRollAngles();
    }

    // --- Intake Controls ---

    public void setIntakePower(double power) {
        intakeLower.setPower(power);
    }

    public void stopIntake() {
        setIntakePower(0.0);
    }

    // --- Flywheel Controls ---

    public void setFlywheelPower(double power) {
        flywheel.setPower(power);
    }

    public void stopFlywheel() {
        flywheel.setPower(0.0);
    }
    public void setBumperSpeedNull() {
        bumperServo.setPower(0.0);
    }
    public void setBumperClockwiseMax() {
        bumperServo.setDirection(DcMotorSimple.Direction.FORWARD);
        bumperServo.setPower(1.0);
    }
    public void setBumperCounterClockwiseMax() {
        bumperServo.setDirection(DcMotorSimple.Direction.REVERSE); 
        bumperServo.setPower(1.0);  // full speed
    }
}
