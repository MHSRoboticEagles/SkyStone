package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.skills.DetectionInterface;
import org.firstinspires.ftc.teamcode.skills.StoneFinder;


@Autonomous(name="1Stone Blue", group ="Robot15173")
//@Disabled
public class StoneOneBlue extends AutoBase {
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

            robot.getGyro().turn(-85, 0.7, 0,this, new DetectionInterface() {
                @Override
                public boolean detect() {
                    return robot.autoLockStone();
                }
            });

            if (robot.autoLockStone()) {
                robot.moveIntake(0);
            }

            robot.getGyro().fixHeading(0.3, this);

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
            robot.getGyro().pivotBackReverse(-165, .8, this);
            move(0.8, -13);

            //release the stone and turn the holder inward
            robot.toggleStoneLock(false);
            robot.swivelStone(false);

            robot.hookTraySide(false, true);
            // start removing the crane
            robot.preMoveCrane(1, -10);

            //turn the tray
            robot.getGyro().turn(-85, .9, 2000,this);
            //unhook the tray
            robot.hookTray(false);

            //push the tray forward to the wall
            move(0.8, 10, 1000);
            robot.getGyro().turn(-90, .9, 1000,this);
            move(0.8, -5, 400);

            //measure distance to the red alliance wall
            sleep(200);
            double toWall = robot.getRangetoObstacleRight();

            // stop crane move
            robot.postMoveCrane();

            //fix the original heading of 85 degrees (Is there any way that we can put a timeout on this. It is going in an endless loop)
            robot.getGyro().fixHeading(0.45, this);

            move(0.9, -35);


            telemetry.addData("Elapsed (ms)", elapsedtime);
//            telemetry.addData("traveled", traveled);
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
