package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.BoBot;


@Autonomous(name = "Blue Alliance Near Zone Auto", group = "Autos")
public class BlueAllianceNearAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        BoBot robot = new BoBot(this);
        waitForStart();

        if (opModeIsActive()) {
            //first is distance, second is speed in ticks per second
            robot.forward(24, 800);
            //sleep for a certain amt of time
            sleep(500);
            robot.right(12, 800);
            sleep(500);
        }
    }
}

