package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.gamefield.GameStats;
import org.firstinspires.ftc.teamcode.skills.StoneFinder;


@Autonomous(name = "Stone Red", group = "Robot15173")
//@Disabled
public class StoneRedOpt extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        runAutoMode();
    }

    @Override
    protected void initRobot() {
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
        try {
            int start = 10;
            int approach = 0;
            int backUp = 0;

            int runToZone = 0;
            robot.hookTray(false);
            boolean found = detectStone(1);
            stopStoneDetection();
            if (!found) {
                skyStoneIndex = 4;
                robot.getGyro().pivotReverse(30, -0.7, this);
                approach = -27;
                backUp = 5;
                runToZone = 59;
            }

            if (found) {
                move(1, -start);
                initRec();
                found = detectStone(2);
                if (found) {
                    skyStoneIndex = 6;
                } else {
                    skyStoneIndex = 5;
                }

                stopStoneDetection();
                switch (skyStoneIndex) {
                    case 6:
                        robot.getGyro().pivotReverse(12, -0.7, this);
                        approach = -15;
                        backUp = 1;
                        runToZone = 43;
                        break;
                    case 5:
                        robot.getGyro().pivotReverse(26, -0.7, this);
                        approach = -15;
                        backUp = 7;
                        runToZone = 48;
                        break;
                }
            }

            //get close to the stone
            move(0.8, approach);
            //turn intake on and move forward
            move(0.5, -20);

            if (robot.autoLockStone()) {
                robot.moveIntake(0);
            }

            //move back
            move(0.8, backUp);

            //turn left toward the building zone
            robot.getGyro().pivot(85, 0.7, this);

            //run to the building zone
            move(0.9, runToZone - 18);

            //approach the wall
            sleep(100);
            moveBackUntil(0.7, 18, 18, true);
            double toWall = robot.getRangetoObstacleLeft();

            //turn toward the tray
            robot.getGyro().turnAndExtend(170, 0.8, false, this);
            robot.toggleStoneLock(true);
//            robot.moveIntake(0);

            //move the linear extrusion out
            robot.preMoveCrane(1, 10);

            //aproach the tray
            sleep(200);
            double pushed = moveBackUntil(0.7, 1, 20, true);

            robot.hookTray(true);

            //make sure the crane is fully extended
            runtime.reset();
            while (!robot.craneExtended()) {
                if(!opModeIsActive() || runtime.seconds() > 6) {
                    break;
                }
            }

            //stop the crane
            robot.postMoveCrane();

            //position the stone
            robot.swivelStone(true);


            //release the stone and turn the holder inward
            robot.toggleStoneLock(false);

            //pull the tray back
            move(0.8, -21);
            robot.swivelStone(false);

            // start removing the crane
            robot.preMoveCrane(1, -10);

            //grab the tray by ine side
            robot.hookTraySide(false, false);

            //turn the tray
            robot.getGyro().turn(90, 0.9, 2100, this);
            //stop the motor of the linear extrusion
            robot.postMoveCrane();

            //unhook the tray
            robot.hookTray(false);

            //push the tray forward to the wall
            move(0.8, 5, 300);

            move(0.8, -2);

            //measure distance to the red alliance wall
            sleep(200);
            toWall = robot.getRangetoObstacleLeft();

            // swerve back on course if needed
            double traveled = -1;
            int head = robot.getGyro().getDesiredHeading();
            if (toWall > -1) {
                traveled = robot.curveToPath(27, 20, toWall, this, false);
            }

            //fix the original heading of 90 degrees
            robot.getGyro().fixHeading(0.3, this);

            //measure the distance to the tray wall and calculate how far to go back
            double back = robot.getRangetoObstacleBack();
            double retreat = 60;
            if (back > -1){
                retreat = retreat - back;
            }
            else{
                if (traveled >= 0){
                    retreat -= GameStats.TILE_WIDTH;
                }
            }

            if (skyStoneIndex == 5){
                retreat += StoneFinder.STONE_WIDTH;
            }
            else if (skyStoneIndex == 4){
                retreat += StoneFinder.STONE_WIDTH * 2;
            }
            double elapsed = getRuntime();
//            //run back after the time check
//            if (getRuntime() > 25){
//                //to park
//                move(0.5, -(48 - back));
//            }
//            else {
//                //all the way
//                move(0.5, -retreat);
//            }

            move(0.9, -retreat);

            robot.getGyro().pivotBackReverse(55, 0.6, this);

            robot.getGyro().pivot(88, 0.7, this);
            move(0.9, retreat - 45);

            telemetry.addData("elapsed", elapsed);
            telemetry.addData("Retreat", retreat);
            telemetry.addData("head", head);
            telemetry.addData("traveled", traveled);
            telemetry.addData("Wall", toWall);
            telemetry.addData("Sky Stone", skyStoneIndex);
//            telemetry.addData("Back", back);
//
            telemetry.update();
            sleep(20000);

        } catch (Exception ex) {
            telemetry.addData("Error", ex.getMessage());
            telemetry.update();
        } finally {
            stopStoneDetection();
        }
    }

}
