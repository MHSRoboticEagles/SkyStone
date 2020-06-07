package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.skills.DetectionInterface;
import org.firstinspires.ftc.teamcode.skills.StoneFinder;


@Autonomous(name="2Stone Red", group ="Robot15173")
@Disabled
public class StoneRed extends AutoBase {
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
        int elapsedtime = 0;
        runtime.reset();
        super.act();
        try {
            int start = 11;
            int approach = 0;
            int backUp = 0;

            int runToZone = 0;
            robot.hookTray(false);
            sleep(300);
            boolean found = detectStone(1);
            stopStoneDetection();
            if (!found) {
                skyStoneIndex = 4;
                robot.getGyro().pivotForward(27, -0.7, this);
                approach = -29;
                backUp = 7;
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
                        // maybe change angle
                        robot.getGyro().pivotForward(13, -0.7, this);
                        approach = -16;
                        backUp = 18;
                        runToZone = 64;
                        break;
                    case 5:
                        // add more of an angle
                        robot.getGyro().pivotForward(28, -0.7, this);
                        approach = -16;
                        backUp = 14;
                        runToZone = 48;
                        break;
                }
            }

            //get close to the stone
            move(0.8, approach);
            //turn intake on and move forward
            robot.moveIntake(1);
            move(0.5, -10);

            //move back
            if (backUp > 0) {
                move(0.8, backUp);
            }

            if (skyStoneIndex == 6){
                robot.getGyro().turn(85, 0.7, this);
            } else if (skyStoneIndex == 5){
                //turn left toward the building zone
                robot.getGyro().pivot(78, 0.7, this, new DetectionInterface() {
                    @Override
                    public boolean detect() {
                        return robot.autoLockStone();
                    }
                });
            } else {
                //turn left toward the building zone
                robot.getGyro().pivot(74, 0.7, this, new DetectionInterface() {
                    @Override
                    public boolean detect() {
                        return robot.autoLockStone();
                    }
                });
            }


            if (robot.autoLockStone()) {
                robot.moveIntake(0);
            }

            //run to the building zone
            move(0.9, runToZone - 10);
            //lock just in case
            robot.toggleStoneLock(true);
            robot.moveIntake(0);

            //approach the wall
            sleep(100);
            moveBackUntil(0.75, 22, 22, true);

            //turn toward the tray
            robot.getGyro().turnAndExtend(170, 0.8, false, this);

            //move the linear extrusion out
            robot.preMoveCrane(1, 10);

            //approach the tray
            sleep(400);
            moveBackUntil(0.7, 1, 15, true);
            // hook tray
            robot.hookTray(true);
            sleep(600);

            //make sure the crane is fully extended
            elapsedtime = (int)runtime.milliseconds();
            runtime.reset();
            while (!robot.craneExtended()) {
                if(!opModeIsActive() || runtime.seconds() > 6) {
                    break;
                }
            }

            elapsedtime += runtime.milliseconds();
            runtime.reset();
            //stop the crane
            robot.postMoveCrane();

            //position the stone
            robot.swivelStone(true);

            //pull the tray back
            robot.getGyro().pivotBackReverse(160, 1, this);
            move(0.8, -12);

            //release the stone and turn the holder inward
            robot.toggleStoneLock(false);
            robot.swivelStone(false);

            // start removing the crane
            robot.preMoveCrane(1, -10);

            // only left hook
            robot.hookTraySide(false, false);

            //turn the tray
            if(skyStoneIndex == 6){
                robot.getGyro().turn(85, 0.7, 2500,this);
            } else {
                robot.getGyro().turn(85, 0.7, 3000,this);
            }

            //unhook the tray
            robot.hookTray(false);

            //push the tray forward to the wall
            move(0.9, 10, 900);
            robot.getGyro().turn(90, 1, 1000,this);
            move(0.9, -5, 400);

            //measure distance to the red alliance wall
            sleep(300);
            double toWall = robot.getRangetoObstacleLeft();

            // stop crane move
            robot.postMoveCrane();

            // swerve back on course if needed
            double traveled = 0;

            if (toWall < 19) {
                if(skyStoneIndex == 4) {
                    traveled = robot.curveToPath(24, 18, toWall, this, false);
                }
            }

            //fix the original heading of 90 degrees
            robot.getGyro().fixHeading(0.3, this);

            if(skyStoneIndex != 5) {
                //measure the distance to the tray wall and calculate how far to go back
                sleep(300);
                double back = robot.getRangetoObstacleBack();
                double retreat = 61;
                if (back > -1) {
                    retreat = retreat - back - 10;
                    if (skyStoneIndex == 6) {
                        retreat += 10;
                    }
                }

                if (skyStoneIndex == 4) {
                    // probably needs to go in more
                    retreat += StoneFinder.STONE_WIDTH * 2;
                    retreat += 10;
                }

                // we move back for second stone
                move(1, -retreat, 3000);

                // start intake
                robot.moveIntake(1);
                // turn into stone
                robot.getGyro().pivotForward(60, -0.8, this);
                // approach stone (add more)
                move(.55, -21);
                // move away from stone
                if(skyStoneIndex == 4){
                    move(.8, 14);
                } else {
                    move(.8, 12);
                }

                // turn back into lane
                robot.getGyro().pivot(90, 0.8, this, new DetectionInterface() {
                    @Override
                    // checks stone to lock
                    public boolean detect() {
                        return robot.autoLockStone();
                    }
                });
                // stop intake
                robot.moveIntake(0);
                // align
                robot.getGyro().fixHeading(0.45, this);

                // calculate distance to bridge and go
                double toBridge = retreat - 40;
                move(0.8, toBridge);

                // check if have stone and enough time left
                elapsedtime += runtime.milliseconds();
                if (elapsedtime <= 26500 && robot.isStoneInside()) {
                    // lock stone again
                    robot.toggleStoneLock(true);
                    // extract crane
                    robot.preMoveCrane(1, 10);
                    // start moving while crane extends
                    move(0.8, 45, 4000);
                    robot.swivelStone(true);

                    runtime.reset();

                    // makes sure crane is extended
                    while (!robot.craneExtended()) {
                        if (!opModeIsActive() || runtime.seconds() > 6) {
                            break;
                        }
                    }

                    // stop crane motors
                    robot.postMoveCrane();
                    // unlock stone
                    robot.toggleStoneLock(false);
                    sleep(100);
                    robot.swivelStone(false);
                    // start retracting crane
                    robot.preMoveCrane(1, -10);
                    // rush back to bridge
                    move(0.9, -35);
                    robot.postMoveCrane();
                }
            } else {
                move(0.9, -35 + (traveled/2));
            }

            //// pause start
            telemetry.addData("Elapsed (ms)", elapsedtime);
//            telemetry.addData("Retreat", retreat);
//            telemetry.addData("traveled", traveled);
            telemetry.addData("Wall", toWall);
            telemetry.addData("Sky Stone", skyStoneIndex);
//            telemetry.addData("Back", back);

            telemetry.update();
            sleep(20000);

            //// ens pause




        }
        catch (Exception ex){
            telemetry.addData("Error", ex.getMessage());
            telemetry.update();
        }
        finally {
            stopStoneDetection();
        }
    }

}
