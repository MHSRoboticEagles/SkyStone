package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Red Simple", group ="Robot15173")

public class RedSimple extends AutoBase {
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
            move(0.5, -12);

            robot.getGyro().correct();

            moveUntil(0.2, 8, -30);
            unfoldIntake();
            robot.intakePressDown();
            moveUntil(0.2, 4, -10);
            robot.pickupTemp(1, telemetry);
            sleep(200);
            move(0.5, 5);
            robot.getGyro().turn(-85, 0.4);
            move(0.8, -55);
            moveUntil(0.4, 20, -40);
            foldIntake();
            robot.getGyro().correct();
            move(0.5, -5);
            unfoldIntake();
            robot.releaseTemp(1, telemetry);
            move(0.5, 2);
            sleep(200);
            foldIntake();
            robot.getGyro().turn(-80, 0.4);
            unfoldIntake();
            move(0.5, 50);

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
