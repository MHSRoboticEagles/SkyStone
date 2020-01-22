package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="CurveTestBack", group ="Robot15173")
//@Disabled
public class CurveTestBack extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        runAutoMode();
    }
    @Override
    protected void initRobot(){
        super.initRobot();
    }

    @Override
    protected void act() {
        super.act();
        try {
            robot.getGyro().turn(90, 0.5);
            robot.getGyro().fixHeading(0.3);
            double toWall = robot.getRangetoObstacleLeft();

            String dir = "to bridge";
            double degrees = 0;


            if (toWall > -1) {

                int longCat = 10;
                if(toWall < 24){
                    double catet = 26-toWall;
                    double travel = Math.sqrt(longCat*longCat + catet * catet);
                    double t = longCat/catet;
                    double rads = Math.atan(t);
                    degrees =  Math.toDegrees(rads);
                    robot.getGyro().turn((int)degrees, 0.5);
                    move(0.5, -travel);
                    robot.getGyro().turn(90, 0.5);
                }

            }

            robot.getGyro().fixHeading(0.3);
            telemetry.addData(dir, degrees);

            telemetry.addData("Wall", toWall);
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
