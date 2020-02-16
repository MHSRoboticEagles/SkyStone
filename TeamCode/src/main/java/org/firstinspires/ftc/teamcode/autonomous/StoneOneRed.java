package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.skills.DetectionInterface;
import org.firstinspires.ftc.teamcode.skills.StoneFinder;


@Autonomous(name="1Stone Red", group ="Robot15173")
//@Disabled
public class StoneOneRed extends AutoBase {
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
            boolean found = detectStone(1);
            stopStoneDetection();
            if (!found) {
                skyStoneIndex = 4;
                robot.getGyro().pivotForward(30, -0.7, this);
                approach = -27;
                backUp = 1;
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
                        robot.getGyro().pivotForward(12, -0.7, this);
                        approach = -15;
                        backUp = 16;
                        runToZone = 64;
                        break;
                    case 5:
                        robot.getGyro().pivotForward(25, -0.7, this);
                        approach = -15;
                        backUp = 2;
                        runToZone = 48;
                        break;
                }
            }

            //get close to the stone
            move(0.8, approach);
            robot.moveIntake(1);
            //turn intake on and move forward
            move(0.5, -10);

            //move back
            if (backUp > 0) {
                move(0.8, backUp);
            }

            if (skyStoneIndex == 6){
                robot.getGyro().turn(85, 0.7, this);
            }
            else {
                //turn left toward the building zone
                robot.getGyro().pivot(85, 0.7, this, new DetectionInterface() {
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
            move(0.9, runToZone - 18);
            //lock just in case
            robot.toggleStoneLock(true);
            robot.moveIntake(0);
            
            //approach the wall
            sleep(100);
            moveBackUntil(0.75, 18, 18, true);

            //turn toward the tray
            robot.getGyro().turnAndExtend(170, 0.8, false, this);

            //move the linear extrusion out
            robot.preMoveCrane(1, 10);

            //approach the tray
            sleep(200);
            moveBackUntil(0.7, 1, 20, true);
            robot.hookTray(true);

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
            robot.getGyro().pivotBackReverse(165, 1, this);
            move(0.8, -14);

            //release the stone and turn the holder inward
            robot.toggleStoneLock(false);
            robot.swivelStone(false);

            // start removing the crane
            robot.hookTraySide(false, false);
            robot.preMoveCrane(1, -10);

            //turn the tray
            robot.getGyro().turn(90, 1, 1500,this);
            //unhook the tray
            robot.hookTray(false);

            //push the tray forward to the wall
            move(0.8, 7, 600);


            //measure distance to the red alliance wall
            sleep(200);
            double toWall = robot.getRangetoObstacleLeft();

            // stop crane move
            robot.postMoveCrane();

            // swerve back on course if needed
            double traveled = 0;

            if (toWall > -1) {
                traveled = robot.curveToPath(26, 18, toWall, this, false);
            }

            //fix the original heading of 90 degrees
            robot.getGyro().fixHeading(0.3, this);

            move(0.9, -40);

            //// pause start
            telemetry.addData("Elapsed (ms)", elapsedtime);
            telemetry.addData("traveled", traveled);
            telemetry.addData("Wall", toWall);
            telemetry.addData("Sky Stone", skyStoneIndex);

            telemetry.update();
            sleep(20000);

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
