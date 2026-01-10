package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.BoBot;


@Autonomous(name = "Red Alliance Near Zone Auto", group = "Autos")
public class RedAllianceNearAuto extends LinearOpMode {
    //i want
    @Override
    public void runOpMode() {
        BoBot robot = new BoBot(this);
        waitForStart();

        if (opModeIsActive()) {
            //first is distance, second is speed in ticks per second
            sleep(100);
            robot.forward(24, 800); // forward is actually backwards, setup the robot's forward facing into the goal w/ two loaded
            //next this line, i want the robot to do the flywheel velocity we do for controlShooting in teleop
            //make a delay of 700 miliseconds with the flywheel still running before doing the next action which regards intake
            //next start the intake with the same speed and funcitonality as selecting the "x" button in main teleop
            // after this occcurs, set both the flywheel velocity and the intake to rest/0
            //next rotate the robot rightwards 45 degrees
            //next strafe right for  a distance that i can change just put the code here
            //next we should make intake again start w/ the speed we call and functionality of selecting x in teleop
            //next go forward by calling robot.backward(); for a distance that i can choose
            //next go backward by calling robot.forward(); for a distance i can choose at the 800 speed that i like
            //rotate the bot leftwards for 45 degrees
            //next this line, i want the robot to do the flywheel velocity we do for controlShooting in teleop
            //make a delay of 700 miliseconds with the flywheel still running before doing the next action which regards intake
            //finally start the intake with the same speed and funcitonality as selecting the "x" button in main teleop
            //strafe right for 15 inches

        }
    }
}

