package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.BoBot;


@Autonomous(name = "Red Alliance Near Zone Auto", group = "Autos")
public class RedAllianceNearAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        BoBot robot = new BoBot(this);
        waitForStart();

        if (opModeIsActive()) {
            //first is distance, second is speed in ticks per second
            sleep(24000);
            robot.forward(24, 800);
            //sleep for a certain amt of time
            sleep(500);
            robot.right(12, 800);
            sleep(500);
        }
    }
}

