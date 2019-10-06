package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Turn Test", group ="Robot15173")
@Disabled
public class TurnTest extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        turn(0.5, 27); //90 degrees   1 = 3 degrees
    }


}
