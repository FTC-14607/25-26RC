package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.BoBot;

@Autonomous(name = "Leave Auto FAR", group = "Autos")
public class LeaveAutoFar extends LinearOpMode {

    @Override
    public void runOpMode() {
        BoBot robot = new BoBot(this);

        waitForStart();
        if (isStopRequested()) return;

        if (opModeIsActive()) {

            sleep(13500);
            if (isStopRequested()) return;
            robot.backward(12, 800);
            if (isStopRequested()) return;


            sleep(200);
        }
    }
}
