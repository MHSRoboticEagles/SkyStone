package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.YellowBot;


public class StraightBase extends LinearOpMode {

    protected YellowBot bot = new YellowBot();
    ElapsedTime timer = new ElapsedTime();

    protected double right = 0;
    protected double left = 0;
    protected double startHead = 0;
    protected double nextHead = 0;
    protected double leftTarget = 0;
    protected double rightTarget = 0;


    private double DISTANCE = 0;

    private double SPEED = bot.CALIB_SPEED;


    public StraightBase(double distance){
        DISTANCE = distance;
    }

    public StraightBase(double distance, double speed){
        DISTANCE = distance;
        SPEED = speed;
    }


    @Override
    public void runOpMode() throws InterruptedException {
        try {
            bot.init(this, hardwareMap, telemetry);
            bot.initGyro();

            telemetry.addData("Status", "Ready for start....");
            telemetry.update();

            waitForStart();

            startHead = bot.getGyroHeading();
            double ratio = moveForward();
            timer.reset();
            while(timer.milliseconds() < 1000 && opModeIsActive()){
                telemetry.addData("Calib","Waiting for gyro to settle ...");
                telemetry.update();
            }
            nextHead = bot.getGyroHeading();

            while (opModeIsActive()) {

                telemetry.addData("leftLong", left);
                telemetry.addData("rightLong", right);
                telemetry.addData("ratio", ratio);

                telemetry.addData("leftTarget", leftTarget);
                telemetry.addData("righTarget", rightTarget);

                telemetry.addData("leftDelay", (left -leftTarget)/leftTarget);
                telemetry.addData("righDelay", (right - rightTarget)/rightTarget);

                telemetry.addData("leftDiff", bot.getLeftOdemeter() - leftTarget);
                telemetry.addData("rightDiff", bot.getRightOdemeter() - rightTarget);

                telemetry.addData("startHead", startHead);
                telemetry.addData("nextHead", nextHead);

                telemetry.update();
            }
        }
        catch (Exception ex){
            telemetry.addData("Error", ex.getMessage());
            telemetry.update();
        }
    }

    private double moveForward(){
        double leftStart = bot.getLeftOdemeter();
        double rightStart = bot.getRightOdemeter();
        leftTarget = bot.getLeftTarget(DISTANCE);
        rightTarget = bot.getRightTarget(DISTANCE);
        bot.moveTo(SPEED, SPEED, DISTANCE);
        this.left = bot.getLeftOdemeter() - leftStart;
        this.right = bot.getRightOdemeter() - rightStart;

        double ratio = right/left;
        return ratio;

    }


}
