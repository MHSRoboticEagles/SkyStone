package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.bots.YellowBot;
import org.firstinspires.ftc.teamcode.gamefield.FieldStats;

import java.util.concurrent.TimeUnit;


@TeleOp(name="MasterCali", group="Robot15173")
public class MasterCali extends LinearOpMode {

    private YellowBot bot = new YellowBot();
    ElapsedTime timer = new ElapsedTime();


    private double strafeLeftReduction = 1;
    private double speedStepLeft = 0.05;

    private double strafeRightReduction = 1;
    private double speedStepRight = 0.05;

    private boolean strafeLeftReducing = true;
    private boolean strafeRightReducing = true;

    private double strafeVeerLeft = -1;
    private double strafeVeerRight = -1;

    boolean strafeDirLeft = true;

    private static double CALIB_SPEED = 0.5;

    private double rightLong = 0;
    private double rightPerDegree = 0;

    private double leftLong = 0;
    private double leftPerDegree = 0;

    private double separation = 0;
    private double horizontalTicksDegree = 0;

    private double actualAngle = 0;


    Deadline gamepadRateLimit;
    private final static int GAMEPAD_LOCKOUT = 500;

    private static final int[] modes = new int[]{0, 1, 2, 3};
    private static final String[] modeNames = new String[]{"Straight", "Spin", "Strafe", "Diag"};

    private int selectedMode = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        try {
            bot.init(this, hardwareMap, telemetry);
            bot.initGyro();
            bot.initCalibData();

            gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);

            telemetry.addData("Master Cali", "Ready to calibrate....");
            telemetry.update();


            waitForStart();
            showStatus();

            while (opModeIsActive()) {
                processCommands();
            }
        }
        catch (Exception ex){
            telemetry.addData("Error", ex.getMessage());
            telemetry.update();
        }
    }

    private void processCommands(){
        if (!gamepadRateLimit.hasExpired()) {
            return;
        }


        if (gamepad1.dpad_down){
            if (selectedMode < modes.length){
                selectedMode++;
            }
            gamepadRateLimit.reset();
            showStatus();
        }

        if (gamepad1.dpad_up){
            if (selectedMode > 0){
                selectedMode--;
            }
            gamepadRateLimit.reset();
            showStatus();
        }

        if (gamepad1.start){
            switch (selectedMode){
                case 0:
                    calibMove();
                    break;
                case 1:
                    calibSpin();
                    break;
                case 2:
                    calibStrafe();
                    break;
                case 3:
                    calibDiag();
                    break;

            }
        }
    }

    private void showStatus(){

        for (int i = 0; i < modes.length; i++) {
            telemetry.addData(modeNames[i], i == selectedMode);
        }

//        telemetry.addData("Selected mode", modeNames[selectedMode]);
        telemetry.update();
    }

    private void calibMove(){
        double leftReduction = 1;
        double rightReduction = 1;
        double leftOdo = bot.getLeftOdemeter();
        double rightOdo = bot.getRightOdemeter();
        bot.moveTo(CALIB_SPEED, CALIB_SPEED, 30);
        double leftDistance = bot.getLeftOdemeter() - leftOdo;
        double rightDistance = bot.getRightOdemeter() - rightOdo;

        if (leftDistance > rightDistance){
            leftReduction = rightDistance/leftDistance;
        }
        if (rightDistance > leftDistance){
            rightReduction = leftDistance/rightDistance;
        }


        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("Calib","Waiting for next step...");
            telemetry.update();
        }

        restoreHead();

        bot.moveTo(CALIB_SPEED*leftReduction, CALIB_SPEED*rightReduction, -30);

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("Calib","Wait...");
            telemetry.update();
        }

        restoreHead();

        telemetry.addData("Calib","Complete");
        telemetry.addData("Left", leftReduction);
        telemetry.addData("Right", rightReduction);
        telemetry.update();

    }

    private void calibSpin(){
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

        telemetry.addData("leftLong", leftLong);
        telemetry.addData("leftPerDegree", leftPerDegree);

        telemetry.addData("rightLong", rightLong);
        telemetry.addData("rightPerDegree", rightPerDegree);

        telemetry.addData("separation", separation);
        telemetry.addData("horizontalTicksDegree", horizontalTicksDegree);
        telemetry.addData("actualAngle", actualAngle);

        telemetry.update();



        //separation
        separation = 2*90 * ((leftLong - rightLong)/actualAngle)/(Math.PI*bot.COUNTS_PER_INCH_REV);
        BotCalibConfig config = bot.getCalibConfig();
        if (config == null){
            config = new BotCalibConfig();
        }

        config.setLeftTickPerDegree(Math.abs(leftPerDegree));
        config.setRightTickPerDegree(Math.abs(rightPerDegree));
        config.setWheelBaseSeparation(Math.abs(separation));
        config.setHorizontalTicksDegree(horizontalTicksDegree);
        ReadWriteFile.writeFile(bot.getCalibConfigFile(), config.serialize());

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("Spin","Calibration complete");
            telemetry.update();
        }

        restoreHead();
    }

    private void calibStrafe(){
        double headChange = 0;

        boolean stop = false;
        while(!stop) {
            headChange = strafe();
            if (strafeVeerRight > -1 && strafeVeerLeft > -1){
                break;
            }

            telemetry.addData("Last turnDegrees", headChange);
            telemetry.addData("strafeLeftReduction", strafeLeftReduction);
            telemetry.addData("strafeRightReduction", strafeRightReduction);
            telemetry.update();
        }
        telemetry.addData("Strafe", "Calibration complete");
        telemetry.addData("Last turnDegrees", headChange);
        telemetry.addData("strafeLeftReduction", strafeLeftReduction);
        telemetry.addData("strafeRightReduction", strafeRightReduction);
        telemetry.update();
        saveConfigStrafe();
    }

    private double strafe(){
        double headChange = 0;
        double currentHead = bot.getGyroHeading();


        if (strafeDirLeft) {
            bot.strafeTo(CALIB_SPEED, 30, strafeDirLeft, strafeLeftReduction);
        }else{
            bot.strafeTo(CALIB_SPEED, 30, strafeDirLeft, strafeRightReduction);
        }


        timer.reset();
        while(timer.milliseconds() < 2000 && opModeIsActive()){
            telemetry.addData("Gyroscope","Stabilizing ...");
            telemetry.update();
        }


        //get change in heading
        double finalHead = bot.getGyroHeading();

        headChange = finalHead - currentHead;


        //if one of the directions has been calibrated, do nothing
        if ((strafeDirLeft && strafeVeerLeft > -1) || (!strafeDirLeft && strafeVeerRight > -1)){
            //do nothing
        }
        else {
            //set values if fully calibrated
            if (headChange >= -2 && headChange <= 2) {
                if (strafeDirLeft) {
                    strafeVeerLeft = strafeLeftReduction;
                } else {
                    strafeVeerRight = strafeRightReduction;
                }
            } else {
                //adjust left motor speed
                if (headChange > 0) {
                    //veering left. reduce left speed
                    if (strafeDirLeft) {
                        if (!strafeLeftReducing) {
                            speedStepLeft = speedStepLeft / 2;
                        }
                        strafeLeftReducing = true;
                        strafeLeftReduction = strafeLeftReduction - speedStepLeft;
                    } else {
                        if (!strafeRightReducing) {
                            speedStepRight = speedStepRight / 2;
                        }
                        strafeRightReducing = true;
                        strafeRightReduction = strafeRightReduction - speedStepRight;
                    }
                } else {
                    if (strafeDirLeft) {
                        if (strafeLeftReducing) {
                            speedStepLeft = speedStepLeft / 2;
                        }
                        strafeLeftReducing = false;
                        strafeLeftReduction = strafeLeftReduction + speedStepLeft;
                        if (strafeLeftReduction > 1){
                            strafeLeftReduction = 1;
                        }
                    } else {
                        if (strafeRightReducing) {
                            speedStepRight = speedStepRight / 2;
                        }
                        strafeRightReducing = false;
                        strafeRightReduction = strafeRightReduction + speedStepRight;
                        if (strafeRightReduction > 1){
                            strafeRightReduction = 1;
                        }
                    }
                }
            }
        }

        timer.reset();
        while(timer.milliseconds() < 3000 && opModeIsActive()){
            telemetry.addData("Cycle","Complete");
            telemetry.addData("strafeDirLeft", strafeDirLeft);
            telemetry.addData("headChange", headChange);
            telemetry.addData("startHead", currentHead);
            telemetry.addData("finalHead", finalHead);
            telemetry.addData("strafeLeftReduction", strafeLeftReduction);
            telemetry.addData("strafeRightReduction", strafeRightReduction);
            telemetry.update();
        }


        //change direction for the next run
        strafeDirLeft = !strafeDirLeft;

        //fix heading and prepare for the next run
        restoreHead();

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("Strafe","Waiting for the next cycle ...");
            telemetry.update();
        }


        return headChange;

    }

    private void calibDiag()
    {
        bot.diagTo(CALIB_SPEED, 30, true);
    }

    private void restoreHead(){
        this.bot.spinH(0, 0.1);
    }

    private void saveConfigStrafe(){
        BotCalibConfig config = bot.getCalibConfig();
        if (config == null){
            config = new BotCalibConfig();
        }

        config.setStrafeLeftReduction(strafeLeftReduction);
        config.setStrafeRightReduction(strafeRightReduction);
        ReadWriteFile.writeFile(bot.getCalibConfigFile(), config.serialize());
    }

}
