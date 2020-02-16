package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.skills.DetectionInterface;


@Autonomous(name="Aim", group ="Robot15173")
@Disabled
public class CraneTest extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        runAutoMode();
    }
    @Override
    protected void initRobot(){
        super.initRobot();
    }

    @Override
    protected void preStart() {
        super.preStart();
        initRec();
    }

    @Override
    protected void act() {
        super.act();
        double run = 75;
        sleep(200);
        double toWall = robot.getRangetoObstacleLeft();

        // swerve back on course if needed
        double traveled = 0;

        if (toWall > -1) {
            traveled = robot.curveToPath(27, 21, toWall, this, false);
        }
        sleep(200);
        double back = robot.getRangetoObstacleBack();
        if (traveled > 0) {
            if (back > -1) {
                run = run - back;
            } else {
                run = run - traveled;
            }
            toWall = robot.getRangetoObstacleLeft();
        }

        detectStoneMove(0.9, -(run));
        robot.moveIntake(1);
        robot.getGyro().pivotForward(-28, -0.8, this);

        move(.55, -15);
        move(.8, 5);
        robot.getGyro().pivot(0, 0.8, this, new DetectionInterface() {
            @Override
            public boolean detect() {
                return robot.autoLockStone();
            }
        });
        robot.getGyro().fixHeading(0.3, this);

        robot.moveIntake(0);


        double walldistance = robot.getRangetoObstacleLeft();
        double backCurve = 0;
        backCurve = robot.curveToPathReverse(29, 24, walldistance, this, false);
        move(0.8, run - 45 - backCurve);
        if (robot.isStoneInside()){
            robot.preMoveCrane(1, 10);
            move(0.8, 40);
            robot.swivelStone(true);
            runtime.reset();
            while (!robot.craneExtended()) {
                if(!opModeIsActive() || runtime.seconds() > 6) {
                    break;
                }
            }
            robot.postMoveCrane();
            robot.toggleStoneLock(false);
            robot.swivelStone(false);
            robot.preMoveCrane(1, -10);
            move(0.9, -40);
            robot.postMoveCrane();
        }



//        telemetry.addData("Sensor Read", reading);
        telemetry.addData("Dist", run);
        telemetry.addData("Back", back);
        telemetry.addData("toWall", toWall);
        telemetry.addData("Walldistance", walldistance);
        telemetry.update();
        sleep(20000);


    }

}
