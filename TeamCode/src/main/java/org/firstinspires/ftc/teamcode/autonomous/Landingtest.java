package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Test Lift", group ="Robot15173")
@Disabled
public class Landingtest extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        runAutoMode();
    }

    @Override
    protected void initRobot(){
        super.initRobot();
        robot.lift.setPower(-1);
    }

    @Override
    protected void act() {
        super.act();
        initArm();
        descend();
        detach();
        postdetach();
        goldPosition = findGold(-1, true);

    }

    @Override
    protected void onGoldDetectionComplete (){
        positionArm();
    }
}
