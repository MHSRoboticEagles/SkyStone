package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Straight24 High", group="Robot15173")
public class Straight24HighSpeed extends StraightBase {


    private static double D = 24;
    private static double power = 1;

    public Straight24HighSpeed(){
        super(D, power);
    }

}
