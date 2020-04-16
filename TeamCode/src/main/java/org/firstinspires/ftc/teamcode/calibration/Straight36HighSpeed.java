package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Straight36 High", group="Robot15173")
public class Straight36HighSpeed extends StraightBase {


    private static double D = 36;
    private static double power = 1;

    public Straight36HighSpeed(){
        super(D, power);
    }

}
