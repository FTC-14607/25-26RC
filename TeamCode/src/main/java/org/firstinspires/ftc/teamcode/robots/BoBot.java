package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

//TODO: Implement auto turning on our robot so that it auto-turns to a specific heading with the press of a button, and then shoots. 
@Config
public class BoBot extends MecanumDrive {



    //bottom intake motor
    public DcMotorEx intakeLower;

    //flywheel motor at the top
    public DcMotorEx flywheel;

    // Binary servo purpose
    public Servo barrierServo;

    //changing this causing a change in ramp angle
    //public Servo hoodServo; -- changed reprogrammed to a continuous servo using axon servo programmer
    public CRServo hoodServo;
    private double rampTarget = 0.0;
    private double rampPosEstimate = 0.0;
    private final ElapsedTime rampTimer = new ElapsedTime();

    //endregion

    //TODO: Make sure to find a good target velocity for near and far.

    //this is from floor(lower one), change based on time it takes as well as slippage
    public static double INTAKE_POWER = 0.6;

    //power used for intake feeding change based on time we have
    public static double INTAKE_FEED_POWER = 0.8;

    // Flywheel velocity targets units: (encoder ticks/sec)
    public static double TARGET_VELO_NEAR = 900;
    public static double TARGET_VELO_FAR  = 1800;

    // Flywheel PIDF
    public static double
            FLYWHEEL_F = 20.5998,
            FLYWHEEL_P = 157.3564,
            FLYWHEEL_I = 0.0,
            FLYWHEEL_D = 0.0;

    // Barrier constants

    public static double BARRIER_OPEN_POS = 1.0;
    public static double BARRIER_CLOSED_POS = 0.2;

    //Ramp constants to tune
    /**
     Make sure to change ranges, these are units i created using change in time, as its not positional anymore
     */
    public static double RAMP_FAR_POS  = 0.0;
    public static double RAMP_NEAR_POS = 1.0;

    // these are the max and min pos with the new units change so there is a hardstop and doesnt break ramp
    public static double RAMP_MIN_POS = -10.0;
    public static double RAMP_MAX_POS =  10.0;

    // ramp controller tuning
    public static double RAMP_MOVE_POWER = 0.7;
    public static double RAMP_TOLERANCE  = 0.05; //tolerance might remove/might make more exact
    public static double RAMP_UNITS_PER_SEC_AT_FULL_POWER = 1.0; // estimate scale

    /**
     * Added this as it might be good to have a home target but not sure. ALSO BEFORE U CODE IDEA 1:
     * CREATE A LOOKUP TABLE THAT MATCHES VELOCITY FOR THE FLYWHEEL WITH ANGLES FOR THE ramp so that when the
     * ramp angle changes, so does the flywheel velocity
     * */
    public static boolean RAMP_HOME_ON_INIT = false;
    public static double  RAMP_HOME_POWER = -0.6;
    public static double  RAMP_HOME_TIME_SEC = 0.6;


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
        flywheel     = hardwareMap.get(DcMotorEx.class, "flywheel");
        barrierServo = hardwareMap.get(Servo.class, "barrierServo");
        hoodServo    = hardwareMap.get(CRServo.class, "rampServo");



        intakeLower.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeLower.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        applyFlywheelPIDF();

        imu.resetYaw();
        updateOrientation();

        // default positioning for all these
        setBarrierClosed();
        initRamp();
        stopFlywheel();
        stopIntake();
    }

    // ------------------------------------ CONTROL & SENSORS ----------------------------------------


    public void update() {
        updateOrientation();
    }

    public void updateOrientation() {
        orientation = imu.getRobotYawPitchRollAngles();
    }
    // ------------------------------------ FLYWHEEL----------------------------------------
    private double lastFlywheelP = Double.NaN;
    private double lastFlywheelI = Double.NaN;
    private double lastFlywheelD = Double.NaN;
    private double lastFlywheelF = Double.NaN;
    public void applyFlywheelPIDF() {
        if (FLYWHEEL_P != lastFlywheelP || FLYWHEEL_I != lastFlywheelI ||
                FLYWHEEL_D != lastFlywheelD || FLYWHEEL_F != lastFlywheelF) {
            PIDFCoefficients pidf = new PIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);
            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            lastFlywheelP = FLYWHEEL_P;
            lastFlywheelI = FLYWHEEL_I;
            lastFlywheelD = FLYWHEEL_D;
            lastFlywheelF = FLYWHEEL_F;
        }
    }
    //encoder ticks/sec) for the following methods
    public void setFlywheelVelocity(double velocityTicksPerSec) {
        applyFlywheelPIDF();
        flywheel.setVelocity(velocityTicksPerSec);
    }

    public double getFlywheelVelocity() {
        return flywheel.getVelocity();
    }

    public void stopFlywheel() {
        flywheel.setPower(0.0);
    }
    // ------------------------------------INTAKE ----------------------------------------

    public void setIntakePower(double power) {
        intakeLower.setPower(power);
    }

    public void stopIntake() {
        setIntakePower(0.0);
    }

    // ------------------------------------ BARRIER----------------------------------------
    public void setBarrierClosed() {
        barrierServo.setPosition(BARRIER_CLOSED_POS);
    }

    public void setBarrierOpen() {
        barrierServo.setPosition(BARRIER_OPEN_POS);
    }

    // ------------------------------------ RAMP/HOOD----------------------------------------
    /*
        added the ability to have a home, not needed i believe but just there
    */
    private void initRamp() {
        if (RAMP_HOME_ON_INIT) {
            hoodServo.setPower(clip(RAMP_HOME_POWER, -1.0, 1.0));
            opMode.sleep((int) Math.round(RAMP_HOME_TIME_SEC * 1000.0));
            hoodServo.setPower(0.0);
        } else {
            hoodServo.setPower(0.0);
        }
        // here we start it at the far position
        rampTarget = clip(RAMP_FAR_POS, RAMP_MIN_POS, RAMP_MAX_POS);
        rampPosEstimate = rampTarget;
        rampTimer.reset();
    }

    public void updateRamp() {
        double dt = rampTimer.seconds();
        rampTimer.reset();
        if (dt > 0.25) dt = 0.0;
        double error = rampTarget - rampPosEstimate;
        double power = 0.0;
        if (Math.abs(error) > RAMP_TOLERANCE) {
            power = Math.signum(error) * clip(RAMP_MOVE_POWER, 0.0, 1.0);
        }
        hoodServo.setPower(clip(power, -1.0, 1.0));
        rampPosEstimate += power * dt * RAMP_UNITS_PER_SEC_AT_FULL_POWER;
        rampPosEstimate = clip(rampPosEstimate, RAMP_MIN_POS, RAMP_MAX_POS);
    }
    public double getRampPos() {
        return rampTarget;
    }
    public double getRampPosEstimate() {
        return rampPosEstimate;
    }
    public void setRampPos(double pos) {
        rampTarget = clip(pos, RAMP_MIN_POS, RAMP_MAX_POS);
    }
    public void setRampNear() { setRampPos(RAMP_NEAR_POS); }
    public void setRampFar()  { setRampPos(RAMP_FAR_POS); }
    public void stopRamp() {
        rampTarget = rampPosEstimate;
        hoodServo.setPower(0.0);
    }
    //endregion
}
