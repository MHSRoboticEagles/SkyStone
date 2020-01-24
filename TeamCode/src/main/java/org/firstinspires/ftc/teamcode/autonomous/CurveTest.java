package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="CurveTest", group ="Robot15173")
@Disabled
public class CurveTest extends AutoBase {
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
            robot.getGyro().turn(90, 0.5, this);
            robot.getGyro().fixHeading(0.3, this);
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
                    degrees = 90 - Math.toDegrees(rads);
                    robot.getGyro().turn(90 + (int)degrees, 0.5, this);
                    move(0.5, travel);
                    robot.getGyro().turn(90, 0.5, this);
                }
                if (toWall > 28){
                    dir = "toWall";
                    double catet = toWall - 26;
                    double travel = Math.sqrt(longCat*longCat + catet * catet);
                    double t = longCat/catet;
                    double rads = Math.atan(t);
                    degrees = Math.toDegrees(rads);
                    robot.getGyro().turn((int)degrees, 0.5, this);
                    move(0.5, travel);
                    robot.getGyro().turn(90, 0.4, this);

                }
            }

            robot.getGyro().fixHeading(0.3, this);
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
