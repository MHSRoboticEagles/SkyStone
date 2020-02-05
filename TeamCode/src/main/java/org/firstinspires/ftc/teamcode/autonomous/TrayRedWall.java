package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="TrayRedWall", group ="Robot15173")
@Disabled
public class TrayRedWall extends AutoBase {
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
            strafeRight(1, 12);
            robot.getGyro().correct();


            moveBackUntil(0.7, 6, 100, true);

            robot.getGyro().correct();
            double tray = robot.getRangetoObstacleBack();
            telemetry.addData("Tray", tray);
            telemetry.update();


            robot.hookTray(true);
            sleep(1000);
            move(0.5, -5);
            robot.getGyro().pivotBack(90, 0.8);
            robot.hookTray(false);
            robot.getGyro().fixHeading(0.3, this);
            move(1, 14);
            moveRightUntil(0.5, 6, true);
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
