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

    private double separation = 0;
    private double horizontalTicksDegree = 0;

    private double actualAngle = 0;
    File calibFile = AppUtil.getInstance().getSettingsFile(BotCalibConfig.BOT_CALIB_CONFIG);


    @Override
    public void runOpMode() throws InterruptedException {
        try {
            bot.init(this, hardwareMap, telemetry);
            bot.initGyro();

            telemetry.addData("Status", "Ready for calibration....");
            telemetry.update();


            waitForStart();

            spinLeft();

            while (opModeIsActive()) {

                telemetry.addData("leftLong", leftLong);
                telemetry.addData("leftPerDegree", leftPerDegree);

                telemetry.addData("rightLong", rightLong);
                telemetry.addData("rightPerDegree", rightPerDegree);

                telemetry.addData("separation", separation);
                telemetry.addData("horizontalTicksDegree", horizontalTicksDegree);
                telemetry.addData("actualAngle", actualAngle);

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
        double horizontalStart = bot.getHorizontalOdemeter();
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
        actualAngle = finalHead - currentHead;

        rightLong = bot.getRightOdemeter();
        rightPerDegree = rightLong / actualAngle;

        leftLong = bot.getLeftOdemeter();
        leftPerDegree = leftLong / actualAngle;

        double horizontalPosition = bot.getHorizontalOdemeter();
        double horizontalShift = horizontalPosition - horizontalStart;

        horizontalTicksDegree = Math.abs(horizontalShift/actualAngle);



        //separation
        separation = 2*90 * ((leftLong - rightLong)/actualAngle)/(Math.PI*bot.COUNTS_PER_INCH_REV);
        BotCalibConfig config = new BotCalibConfig();
        config.setLeftTickPerDegree(Math.abs(leftPerDegree));
        config.setRightTickPerDegree(Math.abs(rightPerDegree));
        config.setWheelBaseSeparation(Math.abs(separation));
        config.setHorizontalTicksDegree(horizontalTicksDegree);
        ReadWriteFile.writeFile(calibFile, config.serialize());
    }

}
