package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="Stone Red", group ="Robot15173")
//@Disabled
public class StoneRedOpt extends AutoBase {
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
        try {
            int start = 10;
            int approach = 0;
            int backUp = 0;

            int runToZone = 0;
            robot.hookTray(false);
            boolean found = detectStone(1);
            stopStoneDetection();
            if (!found){
                skyStoneIndex = 4;
                robot.getGyro().pivotReverse(30, -0.7, this);
                approach = -27;
                backUp = 19;
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
                switch (skyStoneIndex){
                    case 6:
                        robot.getGyro().pivotReverse(12, -0.7, this);
                        approach = -15;
                        backUp = 12;
                        runToZone = 43;
                        break;
                    case 5:
                        robot.getGyro().pivotReverse(26, -0.7, this);
                        approach = -15;
                        backUp = 17;
                        runToZone = 48;
                        break;
                }
            }
            move(0.8, approach);
            //intake on
            move(0.5, -30);

            if (robot.autoLockStone()){
                robot.moveIntake(0);
            }
           move(0.8, backUp);
           robot.getGyro().pivot(85, 0.7, this);
           move(0.9, runToZone);
           robot.getGyro().turnAndExtend(170, 0.8, false, this);

           sleep(200);
           double back = robot.getRangetoObstacleBack();


            moveBackUntil(0.6, 1, 15, true);

            robot.hookTray(true);
            sleep(600);
//
            move(0.8, -20);

            robot.hookTraySide(false, false);

            robot.getGyro().turn(95, 0.9, this);
            robot.hookTray(false);
            move(0.8, 5);

            double toWall = robot.getRangetoObstacleLeft();
            if(toWall > 27){
                double longCat = 20;
                double catet = toWall - 26;
                double travel = Math.sqrt(longCat*longCat + catet * catet);
                double t = catet/longCat;
                double rads = Math.atan(t);
                double degrees =  Math.toDegrees(rads);
                robot.getGyro().pivotReverse((int)(degrees + 90), -0.7, this);
                move(0.5, -travel);
                robot.getGyro().pivotReverse((int)(90), -0.7, this);
            }

//
//            telemetry.addData("retreat", retreat);
            telemetry.addData("Wall", toWall);
            telemetry.addData("Sky Stone", skyStoneIndex);
            telemetry.addData("Back", back);
//
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
