package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="TrayBlueWall", group ="Robot15173")
@Disabled
public class TrayBlueWall extends AutoBase {
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
            strafeLeft(1, 12);
            robot.getGyro().correct();


            moveBackUntil(0.7, 6, 100, true);

            robot.getGyro().correct();
            double tray = robot.getRangetoObstacleBack();
            telemetry.addData("Tray", tray);
            telemetry.update();


            robot.hookTray(true, telemetry);
            sleep(1000);
            move(0.5, -5);
            robot.getGyro().pivotBack(-90, 0.8);
            robot.hookTray(false, telemetry);
            robot.getGyro().fixHeading(0.3);
            move(1, 14);
//            moveLeftUntil(0.5, 6, true);
            moveBackUntil(0.7, 19, 100, true);

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
