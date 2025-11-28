package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Contest extends MecanumDrive {
    public DcMotorEx a, b, c, d, e, f;

    // a
    // b
    // c
    // d
    // e
    // f

    public Contest(LinearOpMode opmode) {
        super(opmode);

        a  = hardwareMap.get(DcMotorEx.class, "a");
        b  = hardwareMap.get(DcMotorEx.class, "b");
        c  = hardwareMap.get(DcMotorEx.class, "c");
        d  = hardwareMap.get(DcMotorEx.class, "d");
        e  = hardwareMap.get(DcMotorEx.class, "e");
        f  = hardwareMap.get(DcMotorEx.class, "f");


    }
}