package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Stone Red", group ="Robot15173")

public class StoneRed extends AutoBase {
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
    protected void act() {
        super.act();
        try {
            moveBackUntil(0.4, 6, 24, true);
            initRec();
            boolean found = detectStone(2);
            robot.getGyro().turn(10, 0.4);
            if (found){
                skyStoneIndex = 6;
            }
            else {
                found = detectStone(2);
                if (found){
                    skyStoneIndex = 5;
                    robot.getGyro().turn(20, 0.4);
                }
                else{
                    skyStoneIndex = 4;
                    robot.getGyro().turn(30, 0.4);
                }
            }

//            robot.moveIntake(1, telemetry);
            moveUntil(0.4, 10, 60, true);
            move(0.2, -15);

            robot.getGyro().pivot(90, 0.8);
            robot.getGyro().fixHeading(0.3);
//            robot.moveIntake(0, telemetry);

//            int num = 6;
//            if (found) {
//                if (num == 5) {
//                    moveBackUntil(0.4, 24, 60, true);
//                    robot.getGyro().correct();
//                    robot.getGyro().turn(35, 0.4);
//                }
//                else { // 6
//                    moveBackUntil(0.4, 25, 60, true);
//                    robot.getGyro().correct();
//                }
//            }else{
//                moveBackUntil(0.4, 20, 60, true);
//                robot.getGyro().correct();
//                robot.getGyro().turn(35, 0.4);
//            }
//            robot.moveIntake(1, telemetry);
//            move(0.2, -25);
//            robot.moveIntake(0, telemetry);
            double toWall = robot.getRangetoObstacleLeft();
            double back = robot.getRangetoObstacleBack();
            double head =  robot.getGyro().getHeading();
            telemetry.addData("Wall", toWall);
            telemetry.addData("Back", back);
            telemetry.addData("Found", found);
            telemetry.addData("Top", stoneTop);
            telemetry.addData("Left", stoneLeft);
            telemetry.addData("Head", head);
            telemetry.addData("Sky Stone", skyStoneIndex);
            telemetry.update();

            //Num 4 = 37
            //num 5 = 114 - 118

            //angle
            // 6 = 132 x 188
            // 5 = 124 x 198, 122x202, 122 x 205
            // 4 = 129 x 192, 130x 189, 130 x 189

//            moveUntil(0.5, 8, -30, true);
//            robot.getGyro().turn(45, 0.6);

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
