package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="Stone Red Bridge", group ="Robot15173")
@Disabled
public class StoneRedBridge extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        runAutoMode();
    }
    @Override
    protected void initRobot(){
        super.initRobot();
        double head =  robot.getGyro().getHeading();
        telemetry.addData("Head", head);
        telemetry.update();
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
            boolean found = detectStone(1);
            if (found){
                if (stoneLeft <=3){
                    skyStoneIndex = 5;
                }
                else{
                    skyStoneIndex = 6;
                }
            }
            else{
                skyStoneIndex = 4;
            }
            move(0.4, -start);

            switch (skyStoneIndex){
                case 6:
                    robot.getGyro().turn(10, 0.4);
                    break;
                case 5:
                    robot.getGyro().turn(20, 0.4);
                    break;
                case 4:
                    robot.getGyro().turn(30, 0.4);
                    break;
            }


            intakeStone(0.5,-45);

            move(1, robot.ROBOT_LENGTH*3/4);

            double toWall = robot.getRangetoObstacleLeft();
            telemetry.addData("Wall", toWall);


            robot.getGyro().pivot(90, 0.8);

            if (!stoneInside) {
                stoneInside = robot.isStoneInside(telemetry);
                if (stoneInside) {
                    robot.toggleStoneLock(true, telemetry);
                    robot.moveIntake(0, telemetry);
                }
            }

            //check distance to right wall

            robot.getGyro().fixHeading(0.4);


            move(1, 40);
            double back = robot.getRangetoObstacleBack();
            if (back > 18){
                move(1, back - 18);
            }
            //moveBackUntil(0.7, 18, 50, true);
            robot.getGyro().turn(180, 0.8);
//            robot.getGyro().fixHeading(0.4);
            //check how far from tray first
            back = robot.getRangetoObstacleBack();

            if (back > 2) {
                move(0.7, back - 2);
            }

            robot.hookTray(true, telemetry);
            sleep(500);

//            move(0.5, -7);
            robot.getGyro().pivotBackReverse(85, 0.8);

            robot.hookTray(false, telemetry);
            robot.getGyro().fixHeading(0.3);

            move(1, 22);

            robot.getGyro().correct(0.5);

            back = robot.getRangetoObstacleBack();
            toWall = robot.getRangetoObstacleLeft();

            move(1, -(48 - (back + 18)));

            robot.getGyro().turn(90, 0.7);
            robot.getGyro().fixHeading(0.3);

            move(1, -32);


           // move(0.7, -28);

            double head =  robot.getGyro().getHeading();
            telemetry.addData("stoneInside", stoneInside);
            telemetry.addData("Wall", toWall);
            telemetry.addData("Back", back);
            telemetry.addData("Found", found);
            telemetry.addData("Head", head);
            telemetry.addData("Sky Stone", skyStoneIndex);
            telemetry.addData("stoneLeft", stoneLeft);

            telemetry.update();

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
