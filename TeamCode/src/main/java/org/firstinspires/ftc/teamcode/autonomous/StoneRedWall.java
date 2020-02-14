package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="Stone Red Helper", group ="Robot15173")
@Disabled
public class StoneRedWall extends AutoBase {
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
            robot.getGyro().pivotForward(-90, -0.7, this);
            double moved = intakeStone(0.5,-30);
            robot.getGyro().fixHeading(0.3, this);
            move(0.5, moved );

            if (!stoneInside) {
                stoneInside = robot.isStoneInside();
                if (stoneInside) {
                    robot.toggleStoneLock(true);
                }
            }
            robot.moveIntake(0);

            double back = robot.getRangetoObstacleBack();
            if (back > 3){
                robot.getGyro().fixHeading(0.3, this);
                move(0.7, back - 3);
            }

            robot.getGyro().turn(-3, 0.5, this);
            robot.getGyro().correct(0.3);

            double toWall = robot.getRangetoObstacleLeft();
            robot.align(toWall, 4, 16, true, this);
            robot.getGyro().fixHeading(0.3, this);

            if (!stoneInside) {
                stoneInside = robot.isStoneInside();
                if (stoneInside) {
                    robot.toggleStoneLock(true);
                }
            }

            if(stoneInside) {
                move(0.5, 80);
                robot.preMoveCrane(1, 10);
                robot.getGyro().turn(25, 0.5, this);
                move(0.7, 3);

                runtime.reset();
                while (!robot.craneExtended()) {
                    if(!opModeIsActive() || runtime.seconds() > 6) {
                        break;
                    }
                }
                robot.postMoveCrane();
                robot.swivelStone(true);
                sleep(500);
                robot.toggleStoneLock(false);
                robot.swivelStone(false);
//                move(0.5, -moveback);
                robot.getGyro().turn(5, 0.5, this);
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
