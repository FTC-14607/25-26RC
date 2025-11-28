package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Contest extends MecanumDrive {
    public Servo a, b;
    public DcMotorEx c, d;

    // a, b: intake servos
    // c, d: flywheel motors

    public Contest(LinearOpMode opmode) {
        super(opmode);


    }
}