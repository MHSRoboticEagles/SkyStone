package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="TrayRed", group ="Robot15173")

public class TrayRed extends AutoBase {
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
            move(0.9, -28);
            robot.getGyro().correct();
            unfoldIntake();
            robot.intakePressDown();
            robot.getGyro().pivot(-85, 0.8);
            move(1, -16);
            foldIntake();
            moveUntil(0.6, -20, -24);
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
