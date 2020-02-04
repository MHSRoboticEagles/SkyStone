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
            robot.getGyro().turn(-87, 0.5, this);
            robot.getGyro().fixHeading(0.3, this);
            double toWall = robot.getRangetoObstacleRight();

            String dir = "to bridge";
            double degrees = 0;


            if (toWall > -1) {
                int longCat = 10;
                robot.align(toWall, 25, longCat, false, telemetry, this);
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
