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
        strafeRight(1, 7);
//        move(0.5, -13);
//        float left = detectStone(5);
//        telemetry.addData("SkyStone", String.format(" Left position: %.01f in", left));
//        telemetry.update();
//        if (left >= 0) {
//            strafe(1, -7);
//        }
//        else {
//            strafe(1, 8);
//        }
        sleep(20000);
        stopStoneDetection();
    }

}
