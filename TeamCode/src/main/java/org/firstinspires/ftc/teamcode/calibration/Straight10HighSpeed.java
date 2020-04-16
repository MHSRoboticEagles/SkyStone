package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Straight10 High", group="Robot15173")
public class Straight10HighSpeed extends StraightBase {


    private static double D = 10;
    private static double power = 1;

    public Straight10HighSpeed(){
        super(D, power);
    }

}
