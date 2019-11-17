package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="Red First", group ="Robot15173")
@Disabled
public class RedFirst extends AutoBase {
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
            if (!stoneDetected ) {
                stoneDetected = detectStone(2);
            }
            telemetry.addData("SkyStone", String.format(" Left position: %b in", stoneDetected));
            telemetry.update();

            if (stoneDetected) {
                stopStoneDetection();
                //pick right-most
                strafeRight(0.6, 4);
                robot.getGyro().correct();
            }
            else {
                strafeLeft(0.6, 5);
                robot.getGyro().correct();
                stoneDetected = detectStone(2);
                stopStoneDetection();
                telemetry.addData("SkyStone", String.format(" Left position: %b in", stoneDetected));
                telemetry.update();

                if (stoneDetected){
                    //pick the middle one
                    strafeRight(0.6, 3);
                    robot.getGyro().correct();
                }
                else{
                    //pick left-most
                    strafeLeft(0.6, 3);
                    robot.getGyro().correct();
                }
            }
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
