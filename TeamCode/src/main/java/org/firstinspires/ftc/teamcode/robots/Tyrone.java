package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.hardware.ArmMotor;
import org.firstinspires.ftc.teamcode.util.hardware.Encoder;
import org.firstinspires.ftc.teamcode.util.hardware.LinearSlideMotor;
import org.firstinspires.ftc.teamcode.util.odometry.AutoDriver;
import org.firstinspires.ftc.teamcode.util.odometry.MecanumThreeWheelOdometryDriver;
import org.firstinspires.ftc.teamcode.util.odometry.RobotLocalizer;
import org.firstinspires.ftc.teamcode.util.odometry.ThreeWheelOdometry;
/**
 •  Minimal robot for testing a single motor mapped as "testArm".
 •  This purposely does NOT use mecanum drive—just one motor.
 */
@Config
public class Tyrone extends MecanumDrive {
    public Encoder odoRight; //goBILDA Swingarm Odometry Pod (2000 ticks/rot, 48mm wheel)
    public Encoder odoLeft; //goBILDA Swingarm Odometry Pod (2000 ticks/rot, 48mm wheel)
    public Encoder odoPerp; //goBILDA Swingarm Odometry Pod (2000 ticks/rot, 48mm wheel)

    public static Pose2d currentPose;
    public AutoDriver autoDriver;
    public RobotLocalizer odo;
    public PIDController translationController;
    public PIDController yawController;

    public static double trans_P = 0, trans_I = 0, trans_D = 0, trans_Tol = 0;
    public static double yaw_P = 0, yaw_I = 0, yaw_D = 0, yaw_Tol = 0;
    public static double TRACK_WIDTH = 8.2087;    // 20.85 cm TODO
    public static double CENTER_WHEEL_OFFSET = 0; // 0 cm TODO
    public final static int ODO_TICKS_PER_ROTATION = 2000;
    public final static double ODO_WHEEL_DIAMETER = centimetersToInches(48. / 10);
    public Tyrone(LinearOpMode opmode) {
        super(opmode);
        ROBOT_HEIGHT = 18; //change to actual height of robot
        ROBOT_LENGTH = 18;
        ROBOT_WIDTH = 18;
        WHEEL_DIAMETER = 9.6 / 2.54;
        WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        DRIVETRAIN_TICKS = 537.68984;
        STRAFE_MULTIPLIER = 1.3;
        STRAIGHT_ROTATION_CORRECTION = 0;
        STRAFE_ROTATION_CORRECTION = 0;

        //connect our hardware within our linear op modes
        odoRight = new Encoder(frontLeft);
        odoLeft = new Encoder(backRight);
        odoPerp = new Encoder(frontRight);

        odoLeft.setDirection(Encoder.Direction.REVERSE);
        odo = new ThreeWheelOdometry(
                odoLeft, odoRight, odoPerp,
                TRACK_WIDTH, CENTER_WHEEL_OFFSET, ODO_TICKS_PER_ROTATION, ODO_WHEEL_DIAMETER
        );
        translationController = new PIDController(trans_P, trans_I, trans_D);
        translationController.setTolerance(trans_Tol);

        yawController = new PIDController(yaw_P, yaw_I, yaw_D);
        yawController.setTolerance(yaw_Tol);

        autoDriver = new MecanumThreeWheelOdometryDriver(odo, this, opMode, translationController, yawController);

    }

    public void update(){
        updatePose();
    }
    public void updatePose(){
        autoDriver.updatePose();
        currentPose = autoDriver.getCurrentPose();
    }
    public void updateOrientation() {
        orientation = imu.getRobotYawPitchRollAngles();
    }
    public void resetPose(){
        odo.reset();
        updatePose();
    }
}