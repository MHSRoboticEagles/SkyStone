package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="CurveTestBack", group ="Robot15173")
//@Disabled
public class CurveTestReverse extends AutoBase {
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
            double towall = robot.getRangetoObstacleLeft();
            robot.curveToPathReverse(31, 24, towall, this, false);
            robot.getGyro().fixHeading(0.3, this);
            sleep(20000);
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
