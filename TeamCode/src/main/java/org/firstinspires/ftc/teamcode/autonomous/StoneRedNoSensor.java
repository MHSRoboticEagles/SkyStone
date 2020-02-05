package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="Stone Red", group ="Robot15173")
@Disabled
public class StoneRedNoSensor extends AutoBase {
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
            int runIncrement = 0;
            boolean found = detectStone(1);
            stopStoneDetection();
            if (!found){
                skyStoneIndex = 4;
                runIncrement = 9;
                robot.getGyro().pivotReverse(25, -0.7, this);
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
                        robot.getGyro().turn(10, 0.5, this);
                        break;
                    case 5:
                        robot.getGyro().turn(20, 0.5, this);
                        runIncrement = 2;
                        break;
                }

                intakeStoneNoSensor(0.5,-45);
                move(0.8, 45 - 22);
            }

            if (!stoneInside) {
                stoneInside = robot.isStoneInside();
                if (stoneInside) {
                    robot.toggleStoneLock(true);
                    robot.moveIntake(0);
                }
            }

            robot.getGyro().turn(87, 0.8, this);
            robot.getGyro().fixHeading(0.3, this);

            //todo: check distance to left wall
            sleep(200);
            double toWall = robot.getRangetoObstacleLeft();
            telemetry.addData("Wall", toWall);
            telemetry.update();


            if (toWall > -1) {
                int longCat = 10;

                robot.align(toWall, 26, longCat, true, this);
            }


            robot.getGyro().fixHeading(0.3, this);

            move(1, 60 + runIncrement);

            if (!stoneInside) {
                stoneInside = robot.isStoneInside();
                if (stoneInside) {
                    robot.toggleStoneLock(true);
                    robot.moveIntake(0);
                }
            }

            sleep(200);
            double back = robot.getRangetoObstacleBack();
            if (back > 12){
                move(0.7, back - 12);
            }

            robot.getGyro().turnAndExtend(170, 0.8, stoneInside, this);
            //last check for stone inside


            robot.toggleStoneLock(true);
            robot.moveIntake(0);
            robot.preMoveCrane(1, 10);


            sleep(200);

            back = robot.getRangetoObstacleBack();

            if (back > 1) {
                move(0.7, back - 1);
            }


            runtime.reset();
            while (!robot.craneExtended()) {
                if(!opModeIsActive() || runtime.seconds() > 6) {
                    break;
                }
            }
            robot.postMoveCrane();
            robot.swivelStone(true);




            robot.hookTray(true);
            sleep(500);

            robot.toggleStoneLock(false);
            robot.swivelStone(false);


            robot.moveIntakeReverse(1);


            robot.getGyro().pivotBackReverse(150, 0.7, this);


            move(0.8, -8);

            robot.hookTray(false);
            strafeLeft(1, 5);

            robot.preMoveCrane(1, -10);

            robot.moveIntakeReverse(0);


            sleep(200);
            back = robot.getRangetoObstacleBack();

            if (back > 1) {
                move(0.7, back - 1);
            }

            robot.hookTraySide(true, true);
            sleep(500);

            robot.getGyro().turnBackReverse(90, 0.5, this);
            robot.hookTraySide(false, true);

            robot.postMoveCrane();

            robot.getGyro().fixHeading(0.3, this);

            move(1, 5);
//            move(0.9, -5);
            robot.getGyro().fixHeading(0.3, this);
            sleep(200);
            toWall = robot.getRangetoObstacleLeft();
            double travel = 0;
            if (toWall > -1) {
                int longCat = 10;
                if(toWall < 24){
                    double catet = 26-toWall;
                    travel = Math.sqrt(longCat*longCat + catet * catet);
                    double t = longCat/catet;
                    double rads = Math.atan(t);
                    double degrees =  Math.toDegrees(rads);
                    robot.getGyro().turn((int)degrees, 0.5, this);
                    move(0.9, -travel);
                    robot.getGyro().turn(85, 0.4, this);
                }
            }

            robot.getGyro().fixHeading(0.3, this);

            move(1, -36 + travel);

            telemetry.addData("Wall", toWall);
            telemetry.addData("Sky Stone", skyStoneIndex);
//            telemetry.addData("Back", back);

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
