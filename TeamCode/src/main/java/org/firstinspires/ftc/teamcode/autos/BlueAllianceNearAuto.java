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
            sleep(24000);
            robot.right(8,800);
            robot.forward(48, 800);
            //sleep for a certain amt of time
            sleep(500);
        }
    }
}

