package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Stone Blue", group ="Robot15173")
//@Disabled
public class StoneBlueNoSensor extends AutoBase {
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
                robot.getGyro().pivotReverse(-25, -0.7, this);
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
                        robot.getGyro().turn(-8, 0.5, this);
                        break;
                    case 5:
                        robot.getGyro().turn(-16, 0.5, this);
                        runIncrement = 7;
                        break;
                }

                intakeStoneNoSensor(0.5,-45);
                move(0.8, 45 - 22);
            }

            if (!stoneInside) {
                stoneInside = robot.isStoneInside(telemetry);
                if (stoneInside) {
                    robot.toggleStoneLock(true, telemetry);
                    robot.moveIntake(0, telemetry);
                }
            }

            robot.getGyro().turn(-87, 0.8, this);
            robot.getGyro().fixHeading(0.3, this);

            //check distance to side wall
            sleep(200);
            double toWall = robot.getRangetoObstacleRight();
            telemetry.addData("Wall", toWall);
            telemetry.update();


            if (toWall > -1) {
                int longCat = 10;
                robot.align(toWall, 25, longCat, false, telemetry, this);
            }

            robot.getGyro().fixHeading(0.3, this);

            move(0.8, 60 + runIncrement);

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
                double more = back - 12;
                move(0.7, more);
            }

            robot.getGyro().turnAndExtend(-170, 0.8, stoneInside, this);
            //last check for stone inside

            robot.toggleStoneLock(true, telemetry);
            robot.moveIntake(0, telemetry);
            robot.preMoveCrane(1, 10);

            sleep(200);

            back = robot.getRangetoObstacleBack();

            if (back > 1) {
                move(0.7, back - 1);
            }

            runtime.reset();
            while (!robot.craneExtended(telemetry)) {
                if(!opModeIsActive() || runtime.seconds() > 6) {
                    break;
                }
            }

            robot.postMoveCrane(telemetry);
            robot.swivelStone(true, telemetry);


            robot.hookTray(true,  telemetry);
            sleep(500);

            robot.toggleStoneLock(false, telemetry);
            robot.swivelStone(false, telemetry);


            robot.moveIntakeReverse(1, telemetry);


            robot.getGyro().pivotBackReverse(-150, 0.7, this);

            move(0.8, -8);

            robot.hookTray(false,  telemetry);
            strafeRight(1, 7);

            robot.preMoveCrane(1, -10);

            robot.moveIntakeReverse(0, telemetry);


            back = robot.getRangetoObstacleBack();

            if (back > 1 && back < 12) {
                move(0.7, back - 1);
            }
            else{
                move(0.7, 8);
            }
            robot.hookTraySide(true, false, telemetry);
            sleep(500);

            robot.getGyro().turnBackReverse(-90, 0.5, this);
            robot.hookTraySide(false, false, telemetry);
            robot.postMoveCrane(telemetry);
            robot.getGyro().fixHeading(0.3, this);

            move(0.7, 9);
            //            move(0.9, -5);
            robot.getGyro().fixHeading(0.3, this);
            sleep(200);
            toWall = robot.getRangetoObstacleRight();
            double travel = 0;
            if (toWall > -1) {

                int longCat = 10;
                if(toWall < 24){
                    double catet = 26-toWall;
                    travel = Math.sqrt(longCat*longCat + catet * catet);
                    double t = longCat/catet;
                    double rads = Math.atan(t);
                    double degrees =  Math.toDegrees(rads);
                    robot.getGyro().turn(-(int)degrees, 0.5, this);
                    move(0.5, -travel);
                    robot.getGyro().turn(-85, 0.4, this);
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
