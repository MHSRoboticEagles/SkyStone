package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Stone Blue", group ="Robot15173")

public class StoneBlue extends AutoBase {
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

            moveUntil(0.2, 9, -30);
            unfoldIntake();
            robot.intakePressDown();
            moveUntil(0.2, 4, -10);
            robot.pickupTemp(1, telemetry);
            sleep(200);
            move(0.5, 7);
            robot.getGyro().turn(90, 0.9);
            robot.getGyro().fixHeading(0.3);

            move(0.8, -55);
            foldIntake();
            moveUntil(0.4, 30, -40);
            unfoldIntake();
            robot.releaseTemp(1, telemetry);
            move(0.5, 44);

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
