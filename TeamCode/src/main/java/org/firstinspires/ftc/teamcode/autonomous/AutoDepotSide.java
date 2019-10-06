package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.skills.GoldPosition;

@Autonomous(name="Depot", group ="Robot15173")
public class AutoDepotSide extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        runAutoMode();
    }
    @Override
    protected void initRobot(){
        super.initRobot();
        robot.lift.setPower(-1);
    }

    @Override
    protected void act() {
        super.act();
        initArm();
        descend();
        detach();
        goldPosition = findGold(-1, false);
    }

    @Override
    protected void onGoldDetectionComplete (){
        postdetach();
        positionArm();
        runToTarget();
        runToDepot();
        pulloutFromDepot();
//        runToCrater();
    }
}
