package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="Blue First", group ="Robot15173")
@Disabled
public class BlueFirst extends AutoBase {
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
            boolean rightMost = false;
            move(0.5, -12);

            if (!stoneDetected ) {
                stoneDetected = detectStone(2);
            }
            telemetry.addData("SkyStone", String.format(" Left position: %b in", stoneDetected));
            telemetry.update();

            if (stoneDetected) {
                rightMost = true;
                stopStoneDetection();
                //pick right-most
                strafeRight(0.6, 3);
                robot.getGyro().correct();
            }
            else {
                strafeLeft(0.6, 5);
                robot.getGyro().correct();
                stoneDetected = detectStone(2);
                stopStoneDetection();
                telemetry.addData("SkyStone", String.format(" Left position: %b in", stoneDetected));
                telemetry.update();

                if (stoneDetected) {
                    //pick the middle one
                    strafeRight(0.6, 3);
                    robot.getGyro().correct();
                } else {
                    //pick left-most
                    strafeLeft(0.6, 3);
                    robot.getGyro().correct();
                }
            }




            robot.getGyro().correct();
            if (rightMost) {
                moveUntil(0.2, 8, -30);
                unfoldIntake();
                moveUntil(0.2, 4, -10);
            }
            else{
                move(0.4, -16);
                unfoldIntake();
                move(0.4, -6);
            }
            robot.pickupTemp(1, telemetry);
            sleep(200);
            move(0.5, 5);
            robot.getGyro().turn(80, 0.4);
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
            robot.getGyro().turn(80, 0.4);
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
