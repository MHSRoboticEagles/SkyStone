package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.skills.Parrot;

@Autonomous(name="Sound Test", group ="Robot15173")
@Disabled
public class SoundTest extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        runAutoMode();
    }

    @Override
    protected void act() {
        super.act();
        Parrot p = new Parrot(hardwareMap, telemetry);
        p.playClever();
        while (opModeIsActive()){
            //do nothing
        }
    }
}
