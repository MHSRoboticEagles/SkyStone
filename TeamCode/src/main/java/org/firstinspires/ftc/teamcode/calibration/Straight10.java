package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.YellowBot;


@TeleOp(name="Straight10", group="Robot15173")
public class Straight10 extends StraightBase {


    private static double D = 10;

    public Straight10(){
        super(D);
    }

}
