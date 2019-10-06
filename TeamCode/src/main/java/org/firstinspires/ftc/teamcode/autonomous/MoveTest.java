package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.skills.Parrot;

@Autonomous(name="Test Move Center Depot", group ="Robot15173")
@Disabled
public class MoveTest extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        moveToCenter();
        runToDepotCenter();
    }

}
