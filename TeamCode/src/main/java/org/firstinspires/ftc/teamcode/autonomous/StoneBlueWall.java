package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Stone Blue Helper", group ="Robot15173")
//@Disabled
public class StoneBlueWall extends AutoBase {
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
//            sleep(2000);
            move(0.5, -50);
            robot.getGyro().pivotReverse(90, -0.7, this);
            double moved = intakeStone(0.5,-30);
            robot.getGyro().fixHeading(0.3, this);
            move(0.5, moved );

            if (!stoneInside) {
                stoneInside = robot.isStoneInside(telemetry);
                if (stoneInside) {
                    robot.toggleStoneLock(true, telemetry);
                }
            }
            robot.moveIntake(0, telemetry);

            double back = robot.getRangetoObstacleBack();
            if (back > 3){
                robot.getGyro().fixHeading(0.3, this);
                move(0.7, back - 3);
            }

            robot.getGyro().turn(3, 0.5, this);
            robot.getGyro().correct(0.3);
            double toWall = robot.getRangetoObstacleLeft();
            robot.align(-toWall, 4, 16, telemetry, this);
            robot.getGyro().fixHeading(0.3, this);

            if (!stoneInside) {
                stoneInside = robot.isStoneInside(telemetry);
                if (stoneInside) {
                    robot.toggleStoneLock(true, telemetry);
                }
            }

            if(stoneInside) {
                move(0.5, 80);
                robot.preMoveCrane(1, 10);
                robot.getGyro().turn(-25, 0.5, this);
                move(0.7, 3);
//                double moveback = 5;
//                back = robot.getRangetoObstacleBack();
//                if (back < 10 && back > 3){
//                    moveback = moveback + (back - 3);
//                    move(0.5, back - 3);
//                }
                while (!robot.craneExtended(telemetry)) {

                }
                robot.postMoveCrane(telemetry);
                robot.swivelStone(true, telemetry);
                sleep(500);
                robot.toggleStoneLock(false, telemetry);
                robot.swivelStone(false, telemetry);
//                move(0.5, -moveback);
                robot.getGyro().turn(-5, 0.5, this);
                robot.preMoveCrane(1, -10);
                move(0.5, -25);

            }
            else{
                move(0.5, 20);
            }


            telemetry.addData("Wall", toWall);
            telemetry.addData("Sky Stone", skyStoneIndex);
            telemetry.addData("Back", back);

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
