package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Auto Crater Long", group ="Robot15173")
@Disabled
public class AutoCraterLong extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        runAutoMode();
    }

    @Override
    protected void act() {
        super.act();
        robot.encoderLift(0.5, 1,0, telemetry);
        move(DRIVE_SPEED, 19);

        robot.encoderPivot(PIVOT_SPEED, -5, 0, telemetry);

        move(DRIVE_SPEED, 40);


        robot.encoderPivot(PIVOT_SPEED, -4, 0, telemetry);

        move(DRIVE_SPEED, 45);

        move(DRIVE_SPEED, -100);


    }
}
