package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.bots.YellowBot;

import java.io.File;


@TeleOp(name="Calibration Spin", group="Robot15173")
public class Spin extends LinearOpMode {

    private YellowBot bot = new YellowBot();
    ElapsedTime timer = new ElapsedTime();

    private double rightLong = 0;
    private double rightPerDegree = 0;

    private double leftLong = 0;
    private double leftPerDegree = 0;
    File leftSpinPerDegFile = AppUtil.getInstance().getSettingsFile("leftSpinPerDeg.txt");
    File rightSpinPerDegFile = AppUtil.getInstance().getSettingsFile("rightSpinPerDeg.txt");

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            bot.init(this, hardwareMap, telemetry);
            bot.initGyro();

            telemetry.addData("Status", "Ready for calibration....");
            telemetry.update();




            waitForStart();

            spinLeft();
//            timer.reset();
//            while(timer.milliseconds() < 3000 && opModeIsActive()){
//                telemetry.addData("Calib","Waiting for next step ...");
//                telemetry.update();
//            }
//
//            turnRight();

            while (opModeIsActive()) {

                telemetry.addData("leftLong", leftLong);
                telemetry.addData("leftPerDegree", leftPerDegree);

                telemetry.addData("rightLong", rightLong);
                telemetry.addData("rightPerDegree", rightPerDegree);

                telemetry.update();
            }
        }
        catch (Exception ex){
            telemetry.addData("Error", ex.getMessage());
            telemetry.update();
        }
    }

    private void spinLeft(){
        double currentHead = bot.getGyroHeading();
        double desiredHead = currentHead + 90;
        while (bot.getGyroHeading() < desiredHead && opModeIsActive()){
            if (bot.getGyroHeading() < desiredHead/2){
                bot.spinLeft(bot.CALIB_SPEED, true);
            }else{
                bot.spinLeft(bot.CALIB_SPEED/2, true);
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

        leftLong = bot.getLeftOdemeter();
        leftPerDegree = leftLong / actualAngle;
        ReadWriteFile.writeFile(leftSpinPerDegFile, String.valueOf(leftPerDegree));
        ReadWriteFile.writeFile(rightSpinPerDegFile, String.valueOf(rightPerDegree));
    }

    private void turnRight(){
        double currentHead = bot.getGyroHeading();
        double desiredHead = currentHead - 90;
        while (bot.getGyroHeading() > desiredHead && opModeIsActive()){
            if (bot.getGyroHeading() > desiredHead/2){
                bot.spinRight(bot.CALIB_SPEED, true);
            }else{
                bot.spinRight(bot.CALIB_SPEED/2, true);
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
