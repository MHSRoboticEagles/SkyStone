package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Stone Rec", group ="Robot15173")

public class StoneRecTest extends AutoBase {
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
           moveDetect(0.2, -9);

           if (!stoneDetected){
               stoneDetected = detectStone(2);
           }

           telemetry.addData("Left", stoneLeft);
           telemetry.update();
           sleep(2000);
           if (stoneLeft < 0){
               strafeLeft(1, 5);
           }
           else if (stoneLeft >=0 && stoneLeft < 100){
               strafeRight(1, 3);
           }
           else {
               strafeRight(1, 8);
           }
           sleep(300);
           robot.getGyro().correct();
            moveUntil(0.2, 8, -30);
            unfoldIntake();
            robot.intakePressDown();
            moveUntil(0.2, 4, -10);
            robot.pickupTemp(1, telemetry);
            sleep(200);
            move(0.5, 5);
            robot.getGyro().turn(-90, 0.9);
            robot.getGyro().fixHeading(0.2);

            move(0.8, -55);
            foldIntake();
            moveUntil(0.4, 24, -40);
            unfoldIntake();
            robot.releaseTemp(1, telemetry);
            move(0.5, 50);

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
