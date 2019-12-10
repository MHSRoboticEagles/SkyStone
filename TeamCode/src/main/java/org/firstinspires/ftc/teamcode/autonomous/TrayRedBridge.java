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
            move(0.9, 3);
            move(0.9, -30);
            robot.getGyro().correct();
            robot.getGyro().pivot(-90, 0.8);
            move(1, -16);
            strafeLeft(1, 14);
            robot.getGyro().fixHeading(0.3);
            move(0.8, -10);
            move(0.9, 8);
            move(0.9, 30);

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
