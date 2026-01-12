package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.BoBot;

@Config
@Autonomous(name = "Red Alliance Near Zone Auto", group = "Autos")
public class RedAllianceNearAuto extends LinearOpMode {

    // ------------------ Tunables (change these on dashboard if you use it) ------------------

    // Drive distances (inches) + speed (ticks/sec)
    public static double FIRST_BACK_UP_IN = 34.19;     // you call robot.forward(), but you said it drives backward physically
    public static double DRIVE_SPEED_TPS  = 1000;

    public static double STRAFE_1_RIGHT_IN = 18.18;    // "put the code here" distance
    public static double FORWARD_VIA_BACKWARD_IN = 33.53; // "go forward by calling robot.backward()"
    public static double BACKWARD_VIA_FORWARD_IN = 33.53; // "go backward by calling robot.forward()"

    public static double FINAL_STRAFE_RIGHT_IN = 15;

    // Turning (degrees)
    public static double TURN_RIGHT_DEG = 30;

    // Shooting / feeding timing (ms)
    public static long FLYWHEEL_SPINUP_MS = 1000;    //  700ms delay while flywheel runs
    public static long FEED_MS_1 = 6400;             // how long to run intake to fire the first set
    public static long FEED_MS_2 = 6400;             // how long to run intake to fire the second set

    // Flywheel target (ticks/sec) - matches your FAR teleop
    public static double AUTO_FLYWHEEL_VELO = BoBot.TARGET_VELO_FAR;

    // ---------------------------------------------------------------------------------------

    @Override
    public void runOpMode() {
        BoBot robot = new BoBot(this);

        // Show your requested message during init so the driver can see it before start
        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("forward is actually backwards, setup the robot's forward facing into the goal w/ two loaded");
            telemetry.addLine("This auto will: back up -> shoot -> turn/strafe -> intake+drive -> turn -> shoot -> strafe");
            telemetry.update();
            idle();
        }

        waitForStart();
        if (isStopRequested()) return;

        // ------------------ Step 1: small delay ------------------
        showStep("0) settle (100ms)");
        safeSleep(100);
        if (!opModeIsActive()) return;

        // robot.forward(24, 800)
        showStep("1) robot.forward(" + FIRST_BACK_UP_IN + ", " + DRIVE_SPEED_TPS + ")  (NOTE: forward is physically backwards)");
        robot.forward(FIRST_BACK_UP_IN, DRIVE_SPEED_TPS);
        if (!opModeIsActive()) return;

        //3: flywheel velocity
        showStep("2) spin flywheel @ " + AUTO_FLYWHEEL_VELO);
        robot.setFlywheelVelocity(AUTO_FLYWHEEL_VELO);

        //4: 700ms delay while flywheel runs
        showStep("3) wait " + FLYWHEEL_SPINUP_MS + "ms (flywheel still running)");
        safeSleep(FLYWHEEL_SPINUP_MS);
        if (!opModeIsActive()) return;

        // 5: start intake like pressing X in TeleOp
        showStep("4) start intake (same as X in TeleOp) for " + FEED_MS_1 + "ms");
        robot.setIntakePower(BoBot.INTAKE_POWER);
        safeSleep(FEED_MS_1);
        if (!opModeIsActive()) return;

        //6: stop flywheel + intake
        showStep("5) stop intake + stop flywheel");
        robot.stopIntake();
        robot.stopFlywheel();
        safeSleep(100);
        if (!opModeIsActive()) return;

        // rotate right 45 degrees
        showStep("6) rotate right " + TURN_RIGHT_DEG *-1 + " deg");
        robot.rotate(TURN_RIGHT_DEG*-1);
        if (!opModeIsActive()) return;

        //8: strafe left
        showStep("7) strafe right " + STRAFE_1_RIGHT_IN + " in");
        robot.left(STRAFE_1_RIGHT_IN, DRIVE_SPEED_TPS);
        if (!opModeIsActive()) return;

        //  9: start intake again like X in TeleOp
        showStep("8) start intake again (same as X in TeleOp)");
        robot.setIntakePower(BoBot.INTAKE_POWER);

        // ------------------ Step 10: go forward by calling robot.backward() ------------------
        showStep("9) go forward by calling robot.backward(" + FORWARD_VIA_BACKWARD_IN + ", " + DRIVE_SPEED_TPS + ")");
        robot.backward(FORWARD_VIA_BACKWARD_IN, DRIVE_SPEED_TPS);
        if (!opModeIsActive()) return;
        /*
        // ------------------ Step 11: go backward by calling robot.forward() ------------------
        showStep("10) go backward by calling robot.forward(" + BACKWARD_VIA_FORWARD_IN + ", " + DRIVE_SPEED_TPS + ")");
        robot.forward(BACKWARD_VIA_FORWARD_IN, DRIVE_SPEED_TPS);
        if (!opModeIsActive()) return;

        // (Your screenshot doesn’t explicitly say to stop intake here, but this prevents feeding during the next shoot setup)
        showStep("11) stop intake before second shooting sequence");
        robot.stopIntake();
        safeSleep(100);
        if (!opModeIsActive()) return;

        // ------------------ Step 12: rotate left 45 degrees ------------------
        showStep("12) rotate left " + TURN_RIGHT_DEG + " deg");
        robot.rotate(TURN_RIGHT_DEG);

        if (!opModeIsActive()) return;

        // ------------------ Step 13: flywheel velocity again ------------------
        showStep("13) spin flywheel again @ " + AUTO_FLYWHEEL_VELO);
        robot.setFlywheelVelocity(AUTO_FLYWHEEL_VELO);

        // ------------------ Step 14: 700ms delay while flywheel runs ------------------
        showStep("14) wait " + FLYWHEEL_SPINUP_MS + "ms (flywheel still running)");
        safeSleep(FLYWHEEL_SPINUP_MS);
        if (!opModeIsActive()) return;

        // ------------------ Step 15: start intake like X in TeleOp (feed) ------------------
        showStep("15) start intake (same as X in TeleOp) for " + FEED_MS_2 + "ms");
        robot.setIntakePower(BoBot.INTAKE_POWER);
        safeSleep(FEED_MS_2);
        if (!opModeIsActive()) return;

        // Stop shooter stuff after feeding (keeps it clean + matches your “set to rest/0” idea)
        showStep("16) stop intake + stop flywheel");
        robot.stopIntake();
        robot.stopFlywheel();
        safeSleep(100);
        if (!opModeIsActive()) return;

        // ------------------ Step 16: final strafe right 15 inches ------------------
        showStep("17) final strafe right " + FINAL_STRAFE_RIGHT_IN + " in");
        robot.left(FINAL_STRAFE_RIGHT_IN, DRIVE_SPEED_TPS);
        */
        // End: brake motors
        showStep("DONE");
        robot.brake();
    }

    private void showStep(String msg) {
        telemetry.addLine("forward is actually backwards, setup the robot's forward facing into the goal w/ two loaded");
        telemetry.addData("AUTO STEP", msg);
        telemetry.update();
    }

    /** Sleep that stays responsive to STOP */
    private void safeSleep(long ms) {
        ElapsedTime t = new ElapsedTime();
        t.reset();
        while (opModeIsActive() && !isStopRequested() && t.milliseconds() < ms) {
            idle();
        }
    }
}
