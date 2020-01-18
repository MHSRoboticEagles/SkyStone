package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="TrayRedBridge", group ="Robot15173")

public class TrayRedBridge extends AutoBase {
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


            robot.hookTray(true, telemetry);
            sleep(1000);
            move(0.5, -5);
            robot.getGyro().pivotBack(90, 0.8);
            robot.hookTray(false, telemetry);
            robot.getGyro().fixHeading(0.3);
            move(1, 10);
            moveLeftUntil(0.5, 23, true);
            move(1, 10);
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
