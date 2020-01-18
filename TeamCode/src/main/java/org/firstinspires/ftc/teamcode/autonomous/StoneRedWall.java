package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Stone Red Wall", group ="Robot15173")

public class StoneRedWall extends AutoBase {
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
            double back = robot.getRangetoObstacleBack();
            stopStoneDetection();
            if (!found){
                skyStoneIndex = 4;
                robot.getGyro().pivotReverse(25, -0.7);
                move(1, -24);
                double moved = intakeStone(0.5,-25);
                move(0.5, moved);

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


                switch (skyStoneIndex){
                    case 6:
                        robot.getGyro().turn(10, 0.5);
                        break;
                    case 5:
                        robot.getGyro().turn(20, 0.5);
                        break;
                }

                double moved = intakeStone(0.5,-45);
                move(1, moved - 20);
            }

            robot.getGyro().turn(90, 0.8);

            if (!stoneInside) {
                stoneInside = colorChecker.detect();
                if (stoneInside) {
                    robot.toggleStoneLock(true, telemetry);
                    robot.moveIntake(0, telemetry);
                }
            }

            //check distance to right wall

            robot.getGyro().fixHeading(0.3);

            telemetry.addData("Sky Stone", skyStoneIndex);
            telemetry.addData("Back", back);
//            telemetry.addData("stoneLeft", stoneLeft);

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
