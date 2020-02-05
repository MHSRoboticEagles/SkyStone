package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="CurveTest", group ="Robot15173")
//@Disabled
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
            double currentHead = 0;
            double back = 0;
            if(toWall > 26){
                double longCat = 24;
                double catet = toWall - 26;
                double travel = Math.sqrt(longCat*longCat + catet * catet);
                double t = catet/longCat;
                double rads = Math.atan(t);
                double degrees =  Math.toDegrees(rads);
                robot.getGyro().pivotReverse((int)(degrees + 90), -0.7, this);
                move(0.7, -travel/2);
                currentHead = robot.getGyro().getHeading();
                robot.getGyro().pivotForward(92, -0.8, this);
                robot.stop();
                back = robot.getRangetoObstacleBack();
            }

            telemetry.addData("CurrentHead", currentHead);
            telemetry.addData("Wall", toWall);
            telemetry.addData("back", back);
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
