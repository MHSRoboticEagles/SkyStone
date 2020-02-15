package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.skills.DetectionInterface;

// Priya Test Push

@Autonomous(name="Stone Blue Sensor", group ="Robot15173")
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
                backUp = 16;
                runToZone = 59;
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
                        robot.getGyro().pivotForward(-5, -0.7, this);
                        approach = -15;
                        backUp = 14;
                        runToZone = 64;
                        break;
                    case 5:
                        robot.getGyro().pivotForward(-15, -0.7, this);
                        approach = -15;
                        backUp = 15;
                        runToZone = 48;
                        break;
                }
            }

            //get close to the stone
            move(0.8, approach);
//            robot.moveIntake(1);
            //turn intake on and move forward
            move(0.5, -10);

            //move back
            if (backUp > 0) {
                move(0.8, backUp);
            }


                robot.getGyro().turn(-85, 0.7, 0,this, new DetectionInterface() {
                    @Override
                    public boolean detect() {
                        return robot.autoLockStone();
                    }
                });

            if (robot.autoLockStone()) {
//                robot.moveIntake(0);
            }

            robot.getGyro().fixHeading(0.3, this);

            telemetry.addData("Elapsed (ms)", elapsedtime);
//            telemetry.addData("Retreat", retreat);
//            telemetry.addData("traveled", traveled);
//            telemetry.addData("Wall", toWall);
            telemetry.addData("Sky Stone", skyStoneIndex);
//            telemetry.addData("Back", back);

            telemetry.update();
            sleep(25000);


            //run to the building zone
            move(0.9, runToZone - 18);
            //lock just in case
//            robot.toggleStoneLock(true);
//            robot.moveIntake(0);

            //approach the wall
            sleep(100);
            moveBackUntil(0.75, 18, 18, true);

            //turn toward the tray
            robot.getGyro().turnAndExtend(-170, 0.8, false, this);

            //move the linear extrusion out
//            robot.preMoveCrane(1, 10);

            //approach the tray
            sleep(200);
            moveBackUntil(0.7, 1, 20, true);
            robot.hookTray(true);


            //make sure the crane is fully int
            elapsedtime = (int)runtime.milliseconds();
            runtime.reset();
            while (!robot.craneExtended()) {
                if(!opModeIsActive() || runtime.seconds() > 6) {
                    break;
                }
            }

            elapsedtime += (int)runtime.milliseconds();
            runtime.reset();
            //stop the crane
//            robot.postMoveCrane();

            //position the stone
//            robot.swivelStone(true);

            //pull the tray back
            move(0.8, -20);

            //release the stone and turn the holder inward
//            robot.toggleStoneLock(false);
//            robot.swivelStone(false);

            // start removing the crane
//            robot.preMoveCrane(1, -10);

            //grab the tray by ine side
            robot.hookTraySide(false, false);

            //turn the tray
            robot.getGyro().turn(90, 0.9, 2500, this);
            //stop the motor of the linear extrusion
//            robot.postMoveCrane();

            //unhook the tray
            robot.hookTray(false);

            //push the tray forward to the wall
            move(0.8, 5, 300);

            move(0.8, -1);

            //measure distance to the blue alliance wall
            sleep(200);
            double toWall = robot.getRangetoObstacleRight();








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
