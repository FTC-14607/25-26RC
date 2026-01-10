package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.BoBot;


@Autonomous(name = "Blue Alliance Far Zone Auto", group = "Autos")
public class BlueAllianceFarAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        BoBot robot = new BoBot(this);
        waitForStart();
        //+0.2 so 5.2/5.0 -- 1.04
        if (opModeIsActive()) {
            //first is distance, second is speed in ticks per second
            //dont sleep now
            sleep(500);
            robot.forward(81.9, 800);
            sleep(100);
            robot.rotate(15);
            robot.forward(8,800);

        }
    }
}

