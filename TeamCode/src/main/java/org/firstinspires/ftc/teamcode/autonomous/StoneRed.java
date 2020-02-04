package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="Stone Red Sensor", group ="Robot15173")
@Disabled
public class StoneRed extends AutoBase {
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
                        robot.getGyro().turn(12, 0.5, this);
                        break;
                    case 5:
                        robot.getGyro().turn(22, 0.5, this);
                        runIncrement = 2;
                        break;
                }

                double moved = intakeStone(0.5,-45);
                move(0.8, moved - 22);
            }

            if (!stoneInside) {
                stoneInside = robot.isStoneInside(telemetry);
                if (stoneInside) {
                    robot.toggleStoneLock(true, telemetry);
                    robot.moveIntake(0, telemetry);
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

                robot.align(toWall, 26, longCat, true, telemetry, this);
//                if(toWall < 24){
//                    double catet = 26-toWall;
//                    double travel = Math.sqrt(longCat*longCat + catet * catet);
//                    double t = longCat/catet;
//                    double rads = Math.atan(t);
//                    double degrees = 90 - Math.toDegrees(rads);
//                    robot.getGyro().turn(90 + (int)degrees, 0.5, this);
//                    move(0.5, travel);
//                    robot.getGyro().turn(85, 0.5, this);
//                }
//                if (toWall > 28){
//                    double catet = toWall - 26;
//                    double travel = Math.sqrt(longCat*longCat + catet * catet);
//                    double t = longCat/catet;
//                    double rads = Math.atan(t);
//                    double degrees = Math.toDegrees(rads);
//                    robot.getGyro().turn((int)degrees, 0.5, this);
//                    move(0.5, travel);
//                    robot.getGyro().turn(85, 0.4, this);
//
//                }
            }


            robot.getGyro().fixHeading(0.3, this);

            move(1, 60 + runIncrement);

            if (!stoneInside) {
                stoneInside = robot.isStoneInside(telemetry);
                if (stoneInside) {
                    robot.toggleStoneLock(true, telemetry);
                    robot.moveIntake(0, telemetry);
                }
            }

            sleep(200);
            double back = robot.getRangetoObstacleBack();
            if (back > 12){
//                robot.getGyro().fixHeading(0.3, this);
                move(0.7, back - 12);
            }

            robot.getGyro().turnAndExtend(170, 0.8, stoneInside, this);
            //last check for stone inside
            if (!stoneInside) {
                stoneInside = robot.isStoneInside(telemetry);
                if (stoneInside) {
                    robot.toggleStoneLock(true, telemetry);
                    robot.moveIntake(0, telemetry);
                    robot.preMoveCrane(1, 10);
                }
            }

            sleep(200);

            back = robot.getRangetoObstacleBack();

            if (back > 1) {
                move(0.7, back - 1);
            }

            if (stoneInside){
                while (!robot.craneExtended(telemetry)) {
                    if(!opModeIsActive()) {
                        break;
                    }
                }
                robot.postMoveCrane(telemetry);
                robot.swivelStone(true, telemetry);
            }



            robot.hookTray(true,  telemetry);
            sleep(500);
            if (stoneInside) {
                robot.toggleStoneLock(false, telemetry);
                robot.swivelStone(false, telemetry);

            }
            else{
                robot.moveIntakeReverse(1, telemetry);
            }

            robot.getGyro().pivotBackReverse(150, 0.7, this);

            move(0.8, -16);

            robot.hookTray(false,  telemetry);
            strafeLeft(1, 7);
            if (stoneInside) {
                robot.preMoveCrane(1, -10);
            }
            if (!stoneInside) {
                robot.moveIntakeReverse(0, telemetry);
            }

            sleep(200);
            back = robot.getRangetoObstacleBack();

            if (back > 1) {
                move(0.7, back - 1);
            }

            robot.hookTraySide(true, true, telemetry);
            sleep(500);

            robot.getGyro().turnBackReverse(90, 0.5, this);
            robot.hookTraySide(false, true, telemetry);

            robot.postMoveCrane(telemetry);

            robot.getGyro().fixHeading(0.3, this);

            move(1, 10);
//            move(0.9, -5);
            robot.getGyro().fixHeading(0.3, this);
            sleep(200);
            toWall = robot.getRangetoObstacleLeft();
            double travel = 0;
            if (toWall > -1) {

                int longCat = 20;
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
