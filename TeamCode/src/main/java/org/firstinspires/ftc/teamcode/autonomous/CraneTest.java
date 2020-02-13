package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="Aim", group ="Robot15173")
//@Disabled
public class CraneTest extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        runAutoMode();
    }
    @Override
    protected void initRobot(){
        super.initRobot();
    }

    @Override
    protected void preStart() {
        super.preStart();
        initRec();
    }

    @Override
    protected void act() {
        super.act();
        detectStoneMove(0.9, -75);
        robot.getGyro().pivotForward(-28, -0.8, this);
        robot.getGyro().pivot(85, 0.8, this);
    }

}
