package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.BoBot;


@Autonomous(name = "Red Alliance Far Zone Auto", group = "Autos")
public class RedAllianceFarAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        BoBot robot = new BoBot(this);
        waitForStart();

        if (opModeIsActive()) {
            sleep(24000);
            robot.forward(24, 800);
            sleep(200);
            robot.right(5,800);
            robot.forward(8,800);
        }
    }
}

