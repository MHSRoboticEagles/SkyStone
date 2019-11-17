package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="TrayBlueBridge", group ="Robot15173")

public class TrayBlueBridge extends AutoBase {
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
//            move(0.9, 3);
            move(0.9, -30);
            robot.getGyro().correct();
            unfoldIntake();
            robot.intakePressDown();
            robot.getGyro().pivot(90, 0.8);
            move(1, -16);
            foldIntake();
            strafeRight(1, 21);
            robot.getGyro().fixHeading(0.3);
            move(0.8, -10);
            move(0.9, 8);
            unfoldIntake();
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
