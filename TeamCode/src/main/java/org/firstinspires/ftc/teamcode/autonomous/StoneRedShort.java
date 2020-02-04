package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Stone Red Short", group ="Robot15173")

public class StoneRedShort extends AutoBase {
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
            }


            robot.getGyro().fixHeading(0.3, this);

            move(0.8, 30 + runIncrement);


            robot.toggleStoneLock(true, telemetry);
            robot.moveIntake(0, telemetry);
            robot.preMoveCrane(1, 10);




            runtime.reset();
            while (!robot.craneExtended(telemetry)) {
                if(!opModeIsActive() || runtime.seconds() > 6) {
                    break;
                }
            }
            robot.postMoveCrane(telemetry);
            robot.swivelStone(true, telemetry);


            robot.toggleStoneLock(false, telemetry);
            robot.swivelStone(false, telemetry);

            move(0.7, -10);

            robot.preMoveCrane(1, -10);

            runtime.reset();
            while(true){
                if (runtime.seconds() > 5){
                    break;
                }
            }
            robot.postMoveCrane(telemetry);


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
