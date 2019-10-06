package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.skills.GoldPosition;

@Autonomous(name="Crater", group ="Robot15173")
//@Disabled
public class AutoCraterSide extends AutoBase {
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


    @Override
    protected  void runToDepotCenter() {
        move(DRIVE_SPEED, -14);
        turn(0.5, -27);

        move(DRIVE_SPEED, 40);
        turn(0.5, -17);
        move(DRIVE_SPEED, 30);

        claim();
    }

    @Override
    protected  void runToDepotLeft(){
        move(DRIVE_SPEED, -16);
        turn(0.5, -18);

        move(DRIVE_SPEED, 40);
        turn(0.5, -17);
        move(DRIVE_SPEED, 30);

        claim();
    }

    @Override
    protected  void runToDepotRight(){
        move(DRIVE_SPEED, -16);
        turn(0.5, -36);

        move(DRIVE_SPEED, 40);
        turn(0.5, -17);
        move(DRIVE_SPEED, 30);

        claim();
    }

    @Override
    protected void runToCrater(){
        turn(0.5, -4);
        move(DRIVE_SPEED, -80);
    }

}
