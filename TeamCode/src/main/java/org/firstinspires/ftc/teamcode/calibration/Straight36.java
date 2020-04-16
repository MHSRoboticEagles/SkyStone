package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.YellowBot;


@TeleOp(name="Straight36", group="Robot15173")
public class Straight36 extends StraightBase {

    private static double D = 36;

   public Straight36(){
       super(D);
   }
}