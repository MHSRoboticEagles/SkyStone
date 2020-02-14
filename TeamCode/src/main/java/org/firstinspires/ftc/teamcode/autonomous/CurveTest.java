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
            robot.getGyro().turn(87, 0.8, this);
            sleep(200);
            double toWall = robot.getRangetoObstacleLeft();
            curveToPath(26, 20, toWall);
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

    public void curveToPath(int far, int close, double toWall){
        int head = robot.getGyro().getDesiredHeading();
        double longCat = 24;
        if(toWall > far){
            double catet = toWall - far;
            double travel = Math.sqrt(longCat*longCat + catet * catet);
            double t = catet/longCat;
            double rads = Math.atan(t);
            double degrees =  Math.toDegrees(rads);
            robot.getGyro().pivotForward((int)(degrees + head), -0.7, this);
            move(0.7, -travel/2);
            robot.getGyro().pivotForward(head + 2, -0.8, this);
            robot.stop();
        }
        else if (toWall < close){
            double catet = close - toWall;
            double travel = Math.sqrt(longCat*longCat + catet * catet);
            double t = catet/longCat;
            double rads = Math.atan(t);
            double degrees =  Math.toDegrees(rads);
            robot.getGyro().pivotForward((int)(head - degrees), -0.8, this);
            move(0.7, -travel/2);
            robot.getGyro().pivotForward(head, -0.8, this);
            robot.stop();
        }
    }

}
