package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="Test Move", group ="Robot15173")

public class MoveTest extends AutoBase {
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
            float left = detectStone(2);
            telemetry.addData("SkyStone", String.format(" Left position: %.01f in", left));
            telemetry.update();
            if (left >= 0) {
                strafeRight(0.6, 4.5);
            }
            else {
                strafeLeft(0.6, 6);
            }
            robot.getGyro().correct();
            moveUntil(0.2, 8, -30);
            unfoldIntake();
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
            robot.getGyro().turn(-80, 0.4);
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
