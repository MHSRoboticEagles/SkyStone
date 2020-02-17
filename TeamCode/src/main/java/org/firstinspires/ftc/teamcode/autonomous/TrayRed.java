package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.skills.DetectionInterface;


@Autonomous(name="Tray Red", group ="Robot15173")
//@Disabled
public class TrayRed extends AutoBase {
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
        int elapsedtime = 0;
        runtime.reset();
        super.act();
        try {

            move(0.7, 20);
            robot.getGyro().pivot(90,0.3,this);

            robot.getGyro().fixHeading(0.3, this);
            //approach the tray
            sleep(200);
            moveBackUntil(0.7, 1, 20, true);
            robot.hookTray(true);
            sleep(800);

            //pull the tray back
            robot.getGyro().pivotBackReverse(165, 1, this);
            move(0.8, -14);

            //turn the tray
            robot.getGyro().turn(90, 1, 1500,this);
            //unhook the tray
            robot.hookTray(false);

            //push the tray forward to the wall
            move(0.8, 7, 600);


            //measure distance to the red alliance wall
            sleep(200);
            double toWall = robot.getRangetoObstacleLeft();

            //fix the original heading of 90 degrees
            robot.getGyro().fixHeading(0.3, this);

            move(0.9, -40);

            //// pause start
            telemetry.addData("Elapsed (ms)", elapsedtime);
            telemetry.addData("Wall", toWall);

            telemetry.update();
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
