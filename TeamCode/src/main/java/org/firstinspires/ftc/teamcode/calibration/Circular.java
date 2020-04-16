package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.YellowBot;


@TeleOp(name="Calibration Turn", group="Robot15173")
public class Circular extends LinearOpMode {

    private YellowBot bot = new YellowBot();
    ElapsedTime timer = new ElapsedTime();

    private double rightLong = 0;
    private double rightPerDegree = 0;

    private double leftLong = 0;
    private double leftPerDegree = 0;

    private double nextX = 0;
    private double nextY = 0;

    private double archAngle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            bot.init(this, hardwareMap, telemetry);
            bot.initGyro();

            telemetry.addData("Status", "Ready for calibration....");
            telemetry.update();

            waitForStart();

            turnLeft();
            timer.reset();
            while(timer.milliseconds() < 3000 && opModeIsActive()){
                telemetry.addData("Calib","Waiting for next step ...");
                telemetry.update();
            }

//            turnRight();

            while (opModeIsActive()) {

                telemetry.addData("leftLong", leftLong);
                telemetry.addData("leftPerDegree", leftPerDegree);

                telemetry.addData("X1", nextX);
                telemetry.addData("Y1", nextY);
                telemetry.addData("archAngle", archAngle);

//                telemetry.addData("rightLong", rightLong);
//                telemetry.addData("rightPerDegree", rightPerDegree);

                telemetry.update();
            }
        }
        catch (Exception ex){
            telemetry.addData("Error", ex.getMessage());
            telemetry.update();
        }
    }

    private void turnLeft(){
        double currentHead = bot.getGyroHeading();
        double desiredHead = currentHead + 90;
        while (bot.getGyroHeading() < desiredHead && opModeIsActive()){
            if (bot.getGyroHeading() < desiredHead/2){
                bot.turnLeft(bot.CALIB_SPEED, true);
            }else{
                bot.turnLeft(bot.CALIB_SPEED/2, true);
            }
            telemetry.addData("Heading", bot.getGyroHeading());
            telemetry.update();
        }

        bot.stop();

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("Gyroscope","Stabilizing ...");
            telemetry.update();
        }

        double finalHead = bot.getGyroHeading();
        double actualAngle = finalHead - currentHead;

        rightLong = bot.getRightOdemeter();
        rightPerDegree = rightLong / actualAngle;

        double dLeft = bot.getLeftOdemeter();
        archAngle = (rightLong - dLeft)/bot.ODO_WHEEL_DISTANCE;
        double dCenter = (dLeft + rightLong)/2;
        nextX = bot.START_X + dCenter*Math.cos(currentHead+90);
        nextY = bot.START_Y + dCenter*Math.sin(currentHead+90);
        double head = currentHead+90 + archAngle;
    }

    private void turnRight(){
        double currentHead = bot.getGyroHeading();
        double desiredHead = currentHead - 90;
        while (bot.getGyroHeading() > desiredHead && opModeIsActive()){
            if (bot.getGyroHeading() > desiredHead/2){
                bot.turnRight(bot.CALIB_SPEED, true);
            }else{
                bot.turnRight(bot.CALIB_SPEED/2, true);
            }
            telemetry.addData("Heading", bot.getGyroHeading());
            telemetry.update();
        }

        bot.stop();

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("Gyroscope","Stabilizing ...");
            telemetry.update();
        }

        double finalHead = bot.getGyroHeading();
        double actualAngle = currentHead - finalHead;

        leftLong = bot.getLeftOdemeter();
        leftPerDegree = leftLong / actualAngle;
    }
}
