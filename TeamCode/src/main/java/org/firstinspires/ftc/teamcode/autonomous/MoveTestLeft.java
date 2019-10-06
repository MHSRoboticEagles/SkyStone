package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Move Left", group ="Robot15173")

@Disabled
public class MoveTestLeft extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        moveToLeft();
        runToDepotLeft();
    }

}
