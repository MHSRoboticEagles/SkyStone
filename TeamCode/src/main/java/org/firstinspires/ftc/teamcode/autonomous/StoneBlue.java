package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.skills.DetectionInterface;
import org.firstinspires.ftc.teamcode.skills.StoneFinder;


@Autonomous(name="2Stone Blue", group ="Robot15173")
//@Disabled
public class StoneBlue extends AutoBase {
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
            int start = 12;
            int approach = 0;
            int backUp = 0;

            int runToZone = 0;
            robot.hookTray(false);
            boolean found = detectStone(1);
            stopStoneDetection();
            if (!found) {
                skyStoneIndex = 4;
                robot.getGyro().pivotForward(-20, -0.7, this);
                approach = -27;
                backUp = 14;
                runToZone = 75;
            }

            if (found) {
                move(.9, -start);
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
                        robot.getGyro().pivotForward(-4, -0.7, this);
                        approach = -15;
                        backUp = 13;
                        runToZone = 64;
                        break;
                    case 5:
                        robot.getGyro().pivotForward(-12, -0.7, this);
                        approach = -17;
                        backUp = 17;
                        runToZone = 65;
                        break;
                }
            }

            //get close to the stone
            move(0.8, approach);
            robot.moveIntake(1);
            //turn intake on and move forward
            move(0.5, -12);

            //move back
            if (backUp > 0) {
                move(0.8, backUp + 2);
            }

            robot.getGyro().turn(-90, 0.7, 0,this, new DetectionInterface() {
                @Override
                public boolean detect() {
                    return robot.autoLockStone();
                }
            });

            if (robot.autoLockStone()) {
                robot.moveIntake(0);
            }

            robot.getGyro().fixHeading(0.4, this);

            //run to the building zone
            move(0.9, runToZone - 15);
            //lock just in case
            robot.toggleStoneLock(true);
            robot.moveIntake(0);

            //approach the wall
            sleep(300);
            moveBackUntil(0.75, 16, 14, true);

            //turn toward the tray
            robot.getGyro().turnAndExtend(-170, 0.8, false, this);

            //move the linear extrusion out
            robot.preMoveCrane(1, 10);

            //approach the tray
            sleep(200);
            moveBackUntil(0.7, 1, 20, true);
            move(0.3, -1);

            // grab tray
            robot.hookTray(true);
            sleep(500);

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
            robot.getGyro().pivotBackReverse(-165, .8, this);
            move(0.8, -13);

            //release the stone and turn the holder inward
            robot.toggleStoneLock(false);
            robot.swivelStone(false);

            robot.hookTraySide(false, true);
            // start removing the crane
            robot.preMoveCrane(1, -10);

            //turn the tray
            robot.getGyro().turn(-90, .9, 2500,this);
            //unhook the tray
            robot.hookTray(false);

            //push the tray forward to the wall
            move(0.8, 10, 1000);
            robot.getGyro().turn(-85, .9, 500,this);
            move(0.8, -5, 400);

            //measure distance to the red alliance wall
            sleep(200);
            double toWall = robot.getRangetoObstacleRight();

            // stop crane move
            robot.postMoveCrane();

            //fix the original heading of 85 degrees (Is there any way that we can put a timeout on this. It is going in an endless loop)
            robot.getGyro().fixHeading(0.45, this);

            if(skyStoneIndex != 5) {
                //measure the distance to the tray wall and calculate how far to go back
                sleep(300);
                double back = robot.getRangetoObstacleBack();
                double retreat = 50;
                if (back > -1) {
                    retreat = retreat - back - 10;
                    if (skyStoneIndex == 6) {
                        retreat += 8;
                    }
                }

                if (skyStoneIndex == 4) {
                    retreat += StoneFinder.STONE_WIDTH * 2;
                }

                // we move back for second stone
                move(0.9, -retreat, 3000);

                // start intake
                robot.moveIntake(1);
                // turn into stone
                robot.getGyro().pivotForward(-60, -0.8, this);
                // approach stone
                if (skyStoneIndex == 4){
                    move(.55, -30);
                } else {
                    move(.55, -25);
                }
                // move away from stone
                if (skyStoneIndex == 4){
                    move(.8, 21);
                } else {
                    move(.8, 13);
                }
                // turn back into lane
                robot.getGyro().turn(-90, 0.7, 0,this, new DetectionInterface() {
                    @Override
                    public boolean detect() {
                        return robot.autoLockStone();
                    }
                });
                // stop intake
                robot.moveIntake(0);
                // align
                robot.getGyro().fixHeading(0.45, this);

                // calculate distance to bridge and go
                double toBridge = retreat - 15;
                move(0.8, toBridge);

                robot.getGyro().fixHeading(0.45, this);

                // check if have stone and enough time left
                elapsedtime += runtime.milliseconds();
                if (elapsedtime <= 26500 && robot.isStoneInside()) {
                    // lock stone again
                    robot.toggleStoneLock(true);
                    // extract crane
                    robot.preMoveCrane(1, 10);
                    // start moving while crane extends
                    move(0.8, 48, 4000);
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
                move(0.9, -35);
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
