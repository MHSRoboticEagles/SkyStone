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
                        runIncrement = 2;
                        break;
                }

                double moved = intakeStone(0.5,-45);
                move(1, moved - 22);
            }

            if (!stoneInside) {
                stoneInside = robot.isStoneInside(telemetry);
                if (stoneInside) {
                    robot.toggleStoneLock(true, telemetry);
                    robot.moveIntake(0, telemetry);
                }
            }

            robot.getGyro().turn(90, 0.8);
            robot.getGyro().fixHeading(0.3);

            //todo: check distance to left wall
            double toWall = robot.getRangetoObstacleLeft();
            telemetry.addData("Wall", toWall);
            telemetry.update();


            robot.getGyro().fixHeading(0.3);


            if (toWall > -1) {
                int longCat = 10;
                if(toWall < 24){
                    double catet = 26-toWall;
                    double travel = Math.sqrt(longCat*longCat + catet * catet);
                    double t = longCat/catet;
                    double rads = Math.atan(t);
                    double degrees = 90 - Math.toDegrees(rads);
                    robot.getGyro().turn(90 + (int)degrees, 0.5);
                    move(0.5, travel);
                    robot.getGyro().turn(90, 0.5);
                }
                if (toWall > 28){
                    double catet = toWall - 26;
                    double travel = Math.sqrt(longCat*longCat + catet * catet);
                    double t = longCat/catet;
                    double rads = Math.atan(t);
                    double degrees = Math.toDegrees(rads);
                    robot.getGyro().turn((int)degrees, 0.5);
                    move(0.5, travel);
                    robot.getGyro().turn(90, 0.4);

                }
            }

            toWall = robot.getRangetoObstacleLeft();
            telemetry.addData("Wall", toWall);
            telemetry.update();

            sleep(3000);

            robot.getGyro().fixHeading(0.3);
            move(1, 60 + runIncrement);

            if (!stoneInside) {
                stoneInside = robot.isStoneInside(telemetry);
                if (stoneInside) {
                    robot.toggleStoneLock(true, telemetry);
                    robot.moveIntake(0, telemetry);
                }
            }

            double back = robot.getRangetoObstacleBack();
            if (back > 12){
                robot.getGyro().fixHeading(0.3);
                move(0.7, back - 12);
            }
            toWall = robot.getRangetoObstacleLeft();
            back = robot.getRangetoObstacleBack();
            robot.getGyro().turnAndExtend(170, 0.8, stoneInside);
            //last check for stone inside
            if (!stoneInside) {
                stoneInside = robot.isStoneInside(telemetry);
                if (stoneInside) {
                    robot.toggleStoneLock(true, telemetry);
                    robot.moveIntake(0, telemetry);
                    robot.preMoveCrane(1, 10);
                }
            }

            back = robot.getRangetoObstacleBack();

            if (back > 1) {
                move(0.7, back - 1);
            }

            if (stoneInside){

                while (!robot.craneExtended(telemetry)) {

                }
                robot.postMoveCrane(telemetry);
                robot.swivelStone(true, telemetry);
            }



            robot.hookTray(true,  telemetry);
            sleep(500);
            if (stoneInside) {
                robot.toggleStoneLock(false, telemetry);
                robot.swivelStone(false, telemetry);
                robot.preMoveCrane(1, -10);
            }
            else{
                robot.moveIntakeReverse(1, telemetry);
            }

            robot.getGyro().pivotBackReverse(150, 0.7);

            move(0.8, -16);

            robot.hookTray(false,  telemetry);
            strafeLeft(1, 7);
            if (!stoneInside) {
                robot.moveIntakeReverse(0, telemetry);
            }

            back = robot.getRangetoObstacleBack();

            if (back > 1) {
                move(0.7, back - 1);
            }
            robot.hookTraySide(true, true, telemetry);
            sleep(500);

            robot.getGyro().turnBackReverse(90, 0.5);
            robot.hookTraySide(false, true, telemetry);
            robot.postMoveCrane(telemetry);
            robot.getGyro().fixHeading(0.3);

            move(1, 10);
            move(0.9, -5);
            robot.getGyro().fixHeading(0.3);
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
                    robot.getGyro().turn((int)degrees, 0.5);
                    move(0.5, -travel);
                    robot.getGyro().turn(90, 0.4);
                }
            }

            robot.getGyro().fixHeading(0.3);

            move(1, -40 + travel);

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
