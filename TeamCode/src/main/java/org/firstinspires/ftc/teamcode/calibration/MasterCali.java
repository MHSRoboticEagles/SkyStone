package org.firstinspires.ftc.teamcode.calibration;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.bots.RobotDirection;
import org.firstinspires.ftc.teamcode.bots.RobotMovement;
import org.firstinspires.ftc.teamcode.bots.RobotVeer;
import org.firstinspires.ftc.teamcode.bots.YellowBot;
import org.firstinspires.ftc.teamcode.odometry.RobotCoordinatePostiion;
import org.firstinspires.ftc.teamcode.skills.Led;

import java.util.concurrent.TimeUnit;


@TeleOp(name="MasterCali", group="Robot15173")
public class MasterCali extends LinearOpMode {

    private YellowBot bot = new YellowBot();
    ElapsedTime timer = new ElapsedTime();


    private static double CALIB_SPEED = 0.5;
    private static double CALIB_SPEED_HIGH = 0.9;
    private static double CALIB_SPEED_LOW = 0.2;

    private static double MARGIN_ERROR_DEGREES = 2;


    private double separation = 0;
    private double horizontalTicksDegreeLeft = 0;
    private double horizontalTicksDegreeRight = 0;


    Deadline gamepadRateLimit;
    private final static int GAMEPAD_LOCKOUT = 500;

    private static final int[] modes = new int[]{0, 1, 2, 3, 4, 5};
    private static final String[] modeNames = new String[]{"Straight", "Curve", "Break", "Spin", "Strafe", "Diag"};

    private int selectedMode = 0;

    private boolean speedSettingMode = false;
    private boolean MRSettingMode = false;
    private boolean MRSettingModeBack = false;
    private boolean XSettingMode = false;
    private boolean YSettingMode = false;
    private boolean strafeModeLeft = false;
    private boolean strafeModeRight = false;
    private boolean diagModeLeft = false;
    private boolean diagModeRight = false;
    private boolean startSettingMode = false;

    private boolean tenIncrement = false;

    private int desiredX = 30;
    private int desiredY = 30;
    int startX = 30;
    int startY = 24;
    private static int DIAG = 45;
    private double desiredSpeed = CALIB_SPEED;

    MotorReductionBotCalib templateMRForward = new MotorReductionBotCalib();
    MotorReductionBotCalib templateMRBack = new MotorReductionBotCalib();

    MotorReductionBotCalib templateStrafeLeft = new MotorReductionBotCalib();
    MotorReductionBotCalib templateStrafeRight = new MotorReductionBotCalib();

    MotorReductionBotCalib templateDiagLeft = new MotorReductionBotCalib();
    MotorReductionBotCalib templateDiagRight = new MotorReductionBotCalib();

    private Led led = null;


    @Override
    public void runOpMode() throws InterruptedException {
        try {
            bot.init(this, hardwareMap, telemetry);
            bot.initGyro();
//            bot.initCalibData();

            BotCalibConfig config = bot.getCalibConfig();
            if (config != null){
                this.templateMRForward = new MotorReductionBotCalib(config.getMoveMRForward());
                this.templateMRBack = new MotorReductionBotCalib(config.getMoveMRBack());

                this.templateStrafeLeft = new MotorReductionBotCalib(config.getStrafeLeftReduction());
                this.templateStrafeRight = new MotorReductionBotCalib(config.getStrafeRightReduction());

                this.templateDiagLeft = new MotorReductionBotCalib(config.getDiagMRLeft());
                this.templateDiagRight = new MotorReductionBotCalib(config.getDiagMRRight());
            }
            this.templateMRBack.setDirection(RobotDirection.Backward);

            this.templateStrafeLeft.setDirection(RobotDirection.Left);
            this.templateStrafeRight.setDirection(RobotDirection.Right);

            this.templateDiagLeft.setDirection(RobotDirection.Left);
            this.templateDiagRight.setDirection(RobotDirection.Right);


            this.led = bot.getLights();

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
            sleep(5000);
        }
    }

    private void changeMoveModes(){
        if (!MRSettingMode && !MRSettingModeBack){
            MRSettingMode = true;
        }
        else if (MRSettingMode){
            MRSettingMode = false;
            MRSettingModeBack = true;
        }
        else if (MRSettingModeBack){
            MRSettingMode = false;
            MRSettingModeBack = false;
        }
    }

    private void changeStrafeModes(){
        if (!strafeModeLeft && !strafeModeRight){
            strafeModeLeft = true;
        }
        else if (strafeModeLeft){
            strafeModeLeft = false;
            strafeModeRight = true;
        }
        else if (strafeModeRight){
            strafeModeLeft = false;
            strafeModeRight = false;
        }
    }

    private void changeDiagModes(){
        if (!diagModeLeft && !diagModeRight){
            diagModeLeft = true;
        }
        else if (diagModeLeft){
            diagModeLeft = false;
            diagModeRight = true;
        }
        else if (diagModeRight){
            diagModeLeft = false;
            diagModeRight = false;
        }
    }


    private void processCommands(){
        if (!gamepadRateLimit.hasExpired()) {
            return;
        }


        if(gamepad1.a){
            speedSettingMode = !speedSettingMode;
            gamepadRateLimit.reset();
            showStatus();
        }

        if (gamepad1.b){
            startSettingMode = !startSettingMode;
            gamepadRateLimit.reset();
            showStatus();
        }

        if (gamepad1.x){
            XSettingMode = !XSettingMode;
            gamepadRateLimit.reset();
            showStatus();
        }

        if (gamepad1.y){
            YSettingMode = !YSettingMode;
            gamepadRateLimit.reset();
            showStatus();
        }

        if (gamepad1.left_bumper){
            tenIncrement = !tenIncrement;
            gamepadRateLimit.reset();
        }

        if (gamepad1.back){
            switch (selectedMode) {
                case 0:
                case 1:
                case 2:
                    changeMoveModes();
                    break;
                case 4:
                    changeStrafeModes();
                    break;
                case 5:
                    changeDiagModes();
                    break;
            }

            gamepadRateLimit.reset();
            showStatus();
        }


        if (gamepad1.dpad_down){
            if (speedSettingMode){
                if (desiredSpeed > 0){
                    desiredSpeed = desiredSpeed - 0.1;
                }
            }
            else if (XSettingMode){
                if (startSettingMode){
                    startX = startX - 5;
                }
                else {
                    desiredX = desiredX - 5;
                }
            }
            else if (YSettingMode){
                if (startSettingMode){
                    startY = startY - 5;
                }
                else {
                    desiredY = desiredY - 5;
                }
            }
            else if (MRSettingMode){
                templateMRForward.decrementSelectedMR();
            }
            else if (MRSettingModeBack){
                templateMRBack.decrementSelectedMR();
            }
            else if(strafeModeLeft){
                templateStrafeLeft.decrementSelectedMR();
            }
            else if(strafeModeRight){
                templateStrafeRight.decrementSelectedMR();
            }
            else if(diagModeLeft){
                templateDiagLeft.decrementSelectedMR();
            }
            else if(diagModeRight){
                templateDiagRight.decrementSelectedMR();
            }
            else {
                if (selectedMode < modes.length) {
                    selectedMode++;
                }
            }
            gamepadRateLimit.reset();
            showStatus();
        }

        if (gamepad1.dpad_up){
            if (speedSettingMode){
                if (desiredSpeed < 1){
                    desiredSpeed = desiredSpeed + 0.1;
                }
            }
            else if (XSettingMode){
                if (startSettingMode){
                    startX = startX + 5;
                }
                else {
                    desiredX = desiredX + 5;
                }
            }
            else if (YSettingMode){
                if (startSettingMode){
                    startY = startY + 5;
                }
                else {
                    desiredY = desiredY + 5;
                }
            }
            else if (MRSettingMode){
                templateMRForward.inrementSelectedMR();
            }
            else if (MRSettingModeBack){
                templateMRBack.inrementSelectedMR();
            }
            else if(strafeModeLeft){
                templateStrafeLeft.inrementSelectedMR();
            }
            else if(strafeModeRight){
                templateStrafeRight.inrementSelectedMR();
            }
            else if(diagModeLeft){
                templateDiagLeft.inrementSelectedMR();
            }
            else if(diagModeRight){
                templateDiagRight.inrementSelectedMR();
            }
            else {
                if (selectedMode > 0) {
                    selectedMode--;
                }
            }
            gamepadRateLimit.reset();
            showStatus();
        }
        if (gamepad1.dpad_left){
            if (MRSettingMode){
                templateMRForward.selectPrev();
                gamepadRateLimit.reset();
                showStatus();
            }
            else if(MRSettingModeBack){
                templateMRBack.selectPrev();
                gamepadRateLimit.reset();
                showStatus();
            }
            else if (strafeModeLeft){
                templateStrafeLeft.selectPrev();
                gamepadRateLimit.reset();
                showStatus();
            }
            else if (strafeModeRight){
                templateStrafeRight.selectPrev();
                gamepadRateLimit.reset();
                showStatus();
            }
            else if (diagModeLeft){
                templateDiagLeft.selectPrev();
                gamepadRateLimit.reset();
                showStatus();
            }
            else if (diagModeRight){
                templateDiagRight.selectPrev();
                gamepadRateLimit.reset();
                showStatus();
            }
        }

        if (gamepad1.dpad_right){
            if (MRSettingMode){
                templateMRForward.selectNext();
                gamepadRateLimit.reset();
                showStatus();
            }
            else if(MRSettingModeBack){
                templateMRBack.selectNext();
                gamepadRateLimit.reset();
                showStatus();
            }
            else if (strafeModeLeft){
                templateStrafeLeft.selectNext();
                gamepadRateLimit.reset();
                showStatus();
            }
            else if (strafeModeRight){
                templateStrafeRight.selectNext();
                gamepadRateLimit.reset();
                showStatus();
            }
            else if (diagModeLeft){
                templateDiagLeft.selectNext();
                gamepadRateLimit.reset();
                showStatus();
            }
            else if (diagModeRight){
                templateDiagRight.selectNext();
                gamepadRateLimit.reset();
                showStatus();
            }
        }

        if (gamepad1.start){
            switch (selectedMode){
                case 0:
                    calibMove();
                    break;
                case 1:
                    calibCurve();
                    break;
                case 2:
                    calibBreak();
                    break;
                case 3:
                    calibSpin();
                    break;
                case 4:
                    calibStrafe();
                    break;
                case 5:
                    calibDiag();
                    break;

            }
        }
    }

    private void showStatus(){

        if (speedSettingMode) {
            telemetry.addData("Speed", desiredSpeed);
        }
        else if (XSettingMode){
            if (startSettingMode){
                telemetry.addData("Start X", startX);
            }
            else {
                telemetry.addData("X", desiredX);
            }
        }
        else if (YSettingMode){
            if (startSettingMode){
                telemetry.addData("Start Y", startY);
            }
            else{
                telemetry.addData("Y", desiredY);
            }

        }
        else if (MRSettingMode){
            showMotorReductionCalib(templateMRForward);
        }
        else if (MRSettingModeBack){
            showMotorReductionCalib(templateMRBack);
        }
        else if (strafeModeLeft){
            showMotorReductionCalib(templateStrafeLeft);
        }
        else if (strafeModeRight){
            showMotorReductionCalib(templateStrafeRight);
        }
        else if (diagModeLeft){
            showMotorReductionCalib(templateDiagLeft);
        }
        else if (diagModeRight){
            showMotorReductionCalib(templateDiagRight);
        }
        else {
            for (int i = 0; i < modes.length; i++) {
                telemetry.addData(modeNames[i], i == selectedMode);
            }
        }

        telemetry.update();
    }

    private void calibCurve(){
        RobotCoordinatePostiion locator = null;
        try {
            //tracker
            locator = new RobotCoordinatePostiion(bot, new Point(startX, startY), 75);
            locator.reverseHorEncoder();
            Thread positionThread = new Thread(locator);
            positionThread.start();
            bot.moveToCoordinate(new Point(startX, startY), new Point(desiredX, desiredY), RobotDirection.Forward, desiredSpeed, locator);
            timer.reset();
            while (timer.milliseconds() < 5000 && opModeIsActive()) {

            }
            bot.moveToCoordinate(new Point((int)(Math.round(locator.getXInches())), (int)Math.round(locator.getYInches())), new Point(startX, startY), RobotDirection.Backward, desiredSpeed, locator);
        }
        finally {
            if (locator != null){
                locator.stop();
            }
        }
    }

    private void calibMove(){
        moveBot(templateMRForward, templateMRBack);
        restoreHead();
        led.none();
        saveConfigMove(templateMRForward, templateMRBack);
        showMotorReductionCalib(templateMRForward);
        showMotorReductionCalib(templateMRBack);
        telemetry.update();
    }

    private void moveBot(MotorReductionBotCalib calibF, MotorReductionBotCalib calibB){
        led.none();
        MotorReductionBot mrForward = calibF.getMR();
        MotorReductionBot mrBack = calibB.getMR();

        double currentHead = bot.getGyroHeading();

        double leftOdo = bot.getLeftOdemeter();
        double rightOdo = bot.getRightOdemeter();
        calibF.setLeftOdoDistance(desiredX*bot.COUNTS_PER_INCH_REV);
        calibF.setRightOdoDistance(desiredX*bot.COUNTS_PER_INCH_REV);
        RobotMovement statsF = bot.moveToCalib(desiredSpeed, desiredSpeed, desiredX, mrForward, calibF.getBreakPoint(desiredSpeed), led);
        calibF.setStats(statsF);

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){

        }

        double actualHead = bot.getGyroHeading();
        double headChange = Math.abs(actualHead - currentHead);

        double leftDistance = Math.abs(bot.getLeftOdemeter() - leftOdo);
        double rightDistance = Math.abs(bot.getRightOdemeter() - rightOdo);
        calibF.setLeftOdoDistanceActual(leftDistance);
        calibF.setRightOdoDistanceActual(rightDistance);
        calibF.setHeadChange(headChange);
        calibF.process(false);

        if (Math.abs(headChange) > MARGIN_ERROR_DEGREES){
            led.needAdjustment();
        }
        else{
            led.OK();
        }


        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
        }
        currentHead = bot.getGyroHeading();
        leftOdo = bot.getLeftOdemeter();
        rightOdo = bot.getRightOdemeter();
        calibB.setLeftOdoDistance(desiredX*bot.COUNTS_PER_INCH_REV);
        calibB.setRightOdoDistance(desiredX*bot.COUNTS_PER_INCH_REV);

        RobotMovement statsB =  bot.moveToCalib(desiredSpeed, desiredSpeed, -desiredX, mrBack, calibB.getBreakPoint(desiredSpeed), led);
        calibB.setStats(statsB);

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){

        }

        actualHead = bot.getGyroHeading();
        headChange = Math.abs(actualHead - currentHead);

        calibB.setHeadChange(headChange);

        if (Math.abs(headChange) > MARGIN_ERROR_DEGREES){
            led.needAdjustment();
        }
        else{
            led.OK();
        }

        leftDistance = Math.abs(bot.getLeftOdemeter() - leftOdo);
        rightDistance = Math.abs(bot.getRightOdemeter() - rightOdo);

        calibB.setLeftOdoDistanceActual(leftDistance);
        calibB.setRightOdoDistanceActual(rightDistance);
        calibB.process(false);
    }



    private void calibBreak(){
        BotStatsConfig statsConfig = BotStatsConfig.loadConfig(telemetry);
        int selectedIndex = 0;
        while(selectedIndex < MotorReductionBot.POWER_SAMPLES.length){
            double power = MotorReductionBot.POWER_SAMPLES[selectedIndex];
            RobotMovement statsF =  bot.moveToCalib(power, power, desiredX, templateMRForward, 0, led);
            templateMRForward.setBreakPoint(statsF.getSlowDownDistanceRaw(), power);
            statsConfig.setStatsForward(power, statsF);

            timer.reset();
            while (timer.milliseconds() < 1000 && opModeIsActive()) {

            }
            RobotMovement statsB =  bot.moveToCalib(power, power, -desiredX, templateMRBack, 0, led);
            templateMRBack.setBreakPoint(statsB.getSlowDownDistanceRaw(), power);
            statsConfig.setStatsBack(power, statsB);
            selectedIndex++;

        }
        saveConfigBreak(templateMRForward, templateMRBack);
        statsConfig.saveConfig(telemetry);
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
        double actualAngle = finalHead - currentHead;

        double rightLong = bot.getRightOdemeter();

        double leftLong = bot.getLeftOdemeter();



        double horizontalPosition = bot.getHorizontalOdemeter();
        double horizontalShift = horizontalPosition - horizontalStart;

        horizontalTicksDegreeLeft = Math.abs(horizontalShift/actualAngle);

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("Spin","Will go to right now");
            telemetry.update();
        }

        spinRight();



        //separation
        separation = 2*90 * ((leftLong - rightLong)/actualAngle)/(Math.PI*bot.COUNTS_PER_INCH_REV);

        BotCalibConfig config = bot.getCalibConfig();
        if (config == null){
            config = new BotCalibConfig();
        }

        config.setWheelBaseSeparation(Math.abs(separation));
        config.setHorizontalTicksDegreeLeft(horizontalTicksDegreeLeft);
        config.setHorizontalTicksDegreeRight(horizontalTicksDegreeRight);
        ReadWriteFile.writeFile(bot.getCalibConfigFile(), config.serialize());

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("Spin","Finalizing....");
            telemetry.update();
        }

        restoreHead();

        telemetry.addData("Spin","Calibration complete");

        telemetry.addData("separation", separation);
        telemetry.addData("horizontalTicksDegree Left", horizontalTicksDegreeLeft);
        telemetry.addData("horizontalTicksDegree Right", horizontalTicksDegreeRight);
        telemetry.addData("actualAngle Left", actualAngle);

        telemetry.update();
    }

    private void spinRight(){
        double currentHead = bot.getGyroHeading();
        double desiredHead = currentHead - 90;
        double horizontalStart = bot.getHorizontalOdemeter();
        while (bot.getGyroHeading() > desiredHead && opModeIsActive()){
            if (bot.getGyroHeading() > currentHead/2){
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
        double actualAngle = finalHead - currentHead;


        double horizontalPosition = bot.getHorizontalOdemeter();
        double horizontalShift = horizontalPosition - horizontalStart;

        horizontalTicksDegreeRight = Math.abs(horizontalShift/actualAngle);
    }

    private void calibStrafe(){
        strafeBot(templateStrafeLeft, templateStrafeRight);
        restoreHead();
        led.none();
        saveConfigStrafe(templateStrafeLeft, templateStrafeRight);
        showMotorReductionCalib(templateStrafeLeft);
        showMotorReductionCalib(templateStrafeRight);
        telemetry.update();
    }

    private void strafeBot(MotorReductionBotCalib calibLeft, MotorReductionBotCalib calibRight){
        led.none();
        MotorReductionBot mrLeft = calibLeft.getMR();
        MotorReductionBot mrRight = calibRight.getMR();

        double currentHead = bot.getGyroHeading();

        double leftOdo = bot.getLeftOdemeter();
        double rightOdo = bot.getRightOdemeter();
        bot.strafeToCalib(desiredSpeed, desiredX, true, mrLeft);

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){

        }

        double actualHead = bot.getGyroHeading();
        double headChange = Math.abs(actualHead - currentHead);

        double leftDistance = Math.abs(bot.getLeftOdemeter() - leftOdo);
        double rightDistance = Math.abs(bot.getRightOdemeter() - rightOdo);
        calibLeft.setLeftOdoDistanceActual(leftDistance);
        calibLeft.setRightOdoDistanceActual(rightDistance);
        calibLeft.setHeadChange(headChange);
        calibLeft.process(true);

        if (Math.abs(headChange) > MARGIN_ERROR_DEGREES){
            led.needAdjustment();
        }
        else{
            led.OK();
        }

        restoreHead();

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
        }
        currentHead = bot.getGyroHeading();
        leftOdo = bot.getLeftOdemeter();
        rightOdo = bot.getRightOdemeter();

        bot.strafeToCalib(desiredSpeed, desiredX, false, mrRight);

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){

        }

        actualHead = bot.getGyroHeading();
        headChange = Math.abs(actualHead - currentHead);

        calibRight.setHeadChange(headChange);

        if (Math.abs(headChange) > MARGIN_ERROR_DEGREES){
            led.needAdjustment();
        }
        else{
            led.OK();
        }

        leftDistance = Math.abs(bot.getLeftOdemeter() - leftOdo);
        rightDistance = Math.abs(bot.getRightOdemeter() - rightOdo);

        calibRight.setLeftOdoDistanceActual(leftDistance);
        calibRight.setRightOdoDistanceActual(rightDistance);
        calibRight.process(true);
    }

//    private void calibStrafeMulti(){
//        double headChange = 0;
//
//        boolean stop = false;
//        while(!stop) {
//            headChange = strafe();
//            if (currentMRRight.isCalibComplete() && currentMRLeft.isCalibComplete()){
//                break;
//            }
//        }
//        telemetry.addData("Strafe", "Calibration complete");
//        telemetry.addData("Last turnDegrees", headChange);
//        telemetry.addData("Left Reduction", leftReductionStrafe.toString());
//        telemetry.addData("strafeRightReduction", rightReductionStrafe.toString());
//        telemetry.update();
//        saveConfigStrafe();
//    }

//    private double strafe(){
//        led.none();
//
//        double headChange = 0;
//        double currentHead = bot.getGyroHeading();
//
//        double startLeft = bot.getLeftOdemeter();
//        double startRight = bot.getRightOdemeter();
//
//        bot.strafeToCalib(CALIB_SPEED, desiredX, strafeDirLeft, leftReductionStrafe, rightReductionStrafe);
//
//        MotorReductionList currentList = rightReductionStrafe;
//        MotorReductionCalib currentMRCalib = currentMRRight;
//        if (strafeDirLeft){
//            currentList = leftReductionStrafe;
//            currentMRCalib = currentMRLeft;
//        }
//
//        timer.reset();
//        while(timer.milliseconds() < 2000 && opModeIsActive()){
//            telemetry.addData("Gyroscope","Stabilizing ...");
//            telemetry.update();
//        }
//
//        double endLeft = bot.getLeftOdemeter();
//        double endRight = bot.getRightOdemeter();
//
//        double leftDistance = Math.abs(endLeft - startLeft);
//        double rightDistance = Math.abs(endRight - startRight);
//
//
//        //get change in heading
//        double finalHead = bot.getGyroHeading();
//
//        headChange = finalHead - currentHead;
//
//
//        //if one of the directions has been calibrated, do nothing
//        if (currentMRCalib.isCalibComplete()){
//            //do nothing
//            led.OK();
//        }
//        else {
//            //set values if fully calibrated
//            if (headChange >= -2 && headChange <= 2) {
//                currentMRCalib.setCalibComplete(true);
//                led.OK();
//            } else {
//                led.needAdjustment();
//                double speedReduction = 1;
//                if (leftDistance > rightDistance){
//                    speedReduction = rightDistance/leftDistance;
//                }
//                else{
//                    speedReduction = leftDistance/rightDistance;
//                }
//                //first time
//                if (currentMRCalib.getMotorName() == MotorName.NONE){
//                    currentMRCalib.setMotorName(currentList.getFirsttMR().getMotorName());
//                    currentMRCalib.setOriginalHeadChange(headChange);
//                    currentMRCalib.setHeadChange(headChange);
//                    currentMRCalib.setMotorReduction(speedReduction);
//                }
//                else{
//                    //compare current change in heading with the previous
//                    double oldHeadChange = currentMRCalib.getHeadChange();
//                    boolean not_improved = (oldHeadChange < 0 && headChange <= oldHeadChange) || (oldHeadChange > 0 && headChange >= oldHeadChange);
//                    if (not_improved){
//                        //try another motor next time
//                        currentList.restoreList();
//                        MotorReduction next = currentList.getNextMR(currentMRCalib.getMotorName());
//                        currentMRCalib.setMotorName(next.getMotorName());
//                        currentMRCalib.setMotorReduction(speedReduction);
//                        currentMRCalib.setHeadChange(currentMRCalib.getOriginalHeadChange());
//                    }
//                    else{
//                        double diff = Math.abs(oldHeadChange - headChange);
//                        if (diff < Math.abs(oldHeadChange)){
//                            //need to reduce more
//                            double adjusted = Math.abs(headChange)/diff * currentMRCalib.getMotorReduction();
//                            currentMRCalib.setMotorReduction(adjusted);
//                        }
//                        else{
//                            //overkill. need to reduce less
//                            double adjusted = Math.abs(headChange)/diff * currentMRCalib.getMotorReduction();
//                            currentMRCalib.setMotorReduction(adjusted);
//                        }
//                    }
//                }
//                currentList.updateList(currentMRCalib.getMR());
//            }
//        }
//
//        timer.reset();
//        while(timer.milliseconds() < 3000 && opModeIsActive()){
//            telemetry.addData("Cycle","Complete");
//            telemetry.addData("strafeDirLeft", strafeDirLeft);
//            telemetry.addData("headChange", headChange);
//            telemetry.addData("startHead", currentHead);
//            telemetry.addData("finalHead", finalHead);
//            telemetry.addData("Motor", currentMRCalib.getMotorName());
//            telemetry.addData("Reduction step", currentMRCalib.getReductionStep());
//            telemetry.addData("strafeReduction", currentMRCalib.getMotorReduction());
//            telemetry.update();
//        }
//
//
//        //change direction for the next run
//        strafeDirLeft = !strafeDirLeft;
//
//        //fix heading and prepare for the next run
//        restoreHead();
//
//        timer.reset();
//        while(timer.milliseconds() < 1000 && opModeIsActive()){
//            telemetry.addData("Strafe","Waiting for the next cycle ...");
//            telemetry.update();
//        }
//
//
//        return headChange;
//
//    }
    private void calibDiag(){
        diagBot(templateDiagLeft, templateDiagRight);
        restoreHead();
        led.none();
        //bring bot back
        bot.moveToCalib(CALIB_SPEED, CALIB_SPEED, -desiredX, templateMRBack, 0, this.led);
        saveConfigDiag(templateDiagLeft, templateDiagRight);
        showMotorReductionCalib(templateDiagLeft);
        showMotorReductionCalib(templateDiagRight);
        telemetry.update();
    }

    private void diagBot(MotorReductionBotCalib calibLeft, MotorReductionBotCalib calibRight){
        led.none();
        MotorReductionBot mrLeft = calibLeft.getMR();
        MotorReductionBot mrRight = calibRight.getMR();

        double currentHead = bot.getGyroHeading();

        double leftOdo = bot.getLeftOdemeter();
        double rightOdo = bot.getRightOdemeter();
        bot.diagToCalib(desiredSpeed, 0, desiredX, true, mrLeft);

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){

        }

        double actualHead = bot.getGyroHeading();
        double headChange = Math.abs(actualHead - currentHead);

        double leftDistance = Math.abs(bot.getLeftOdemeter() - leftOdo);
        double rightDistance = Math.abs(bot.getRightOdemeter() - rightOdo);
        calibLeft.setLeftOdoDistanceActual(leftDistance);
        calibLeft.setRightOdoDistanceActual(rightDistance);
        calibLeft.setHeadChange(headChange);
        calibLeft.process(false);

        if (Math.abs(headChange) > MARGIN_ERROR_DEGREES){
            led.needAdjustment();
        }
        else{
            led.OK();
        }

        restoreHead();

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
        }
        currentHead = bot.getGyroHeading();
        leftOdo = bot.getLeftOdemeter();
        rightOdo = bot.getRightOdemeter();

        bot.diagToCalib(desiredSpeed, 0, desiredX, false, mrRight);

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){

        }

        actualHead = bot.getGyroHeading();
        headChange = Math.abs(actualHead - currentHead);

        calibRight.setHeadChange(headChange);

        if (Math.abs(headChange) > MARGIN_ERROR_DEGREES){
            led.needAdjustment();
        }
        else{
            led.OK();
        }

        leftDistance = Math.abs(bot.getLeftOdemeter() - leftOdo);
        rightDistance = Math.abs(bot.getRightOdemeter() - rightOdo);

        calibRight.setLeftOdoDistanceActual(leftDistance);
        calibRight.setRightOdoDistanceActual(rightDistance);
        calibRight.process(false);
    }


//    private void calibDiagMulti(){
//        try {
//            led.none();
//            desiredSpeed = CALIB_SPEED_HIGH;
//
//            MotorReductionCalib calibLeft = new MotorReductionCalib();
//            calibLeft.setCalibSpeed(desiredSpeed);
//            diagMR(true, calibLeft);
//
//            MotorReductionCalib calibRight = new MotorReductionCalib();
//            calibRight.setCalibSpeed(desiredSpeed);
//            diagMR(false, calibRight);
//
//
//            MotorReductionBot mrDiag = new MotorReductionBot();
//
//            if (calibLeft.getMotorName() == MotorName.LB){
//                mrDiag.setLB(calibLeft.getMotorReduction());
//            } else if (calibLeft.getMotorName() == MotorName.RF){
//                mrDiag.setRF(calibLeft.getMotorReduction());
//            }
//
//
//            if (calibRight.getMotorName() == MotorName.LF){
//                mrDiag.setLF(calibRight.getMotorReduction());
//            } else if (calibRight.getMotorName() == MotorName.RB){
//                mrDiag.setRB(calibRight.getMotorReduction());
//            }
//
//            //speed per degree
////            DiagCalibConfig dcLeft = diagAngle(true, calibLeft);
////            DiagCalibConfig dcRight = diagAngle(false, calibRight);
////
////            dcLeft.computeSpeedPerDegree();
////            dcRight.computeSpeedPerDegree();
//
//            telemetry.addData("Speed", desiredSpeed);
//
////            telemetry.addData("Left Angle", dcLeft.getMaxAgle());
////            telemetry.addData("Right Angle", dcRight.getMaxAgle());
////
////            telemetry.addData("Left Speed/degree", dcLeft.getSpeedPerDegree());
////            telemetry.addData("Right Speed/degree", dcRight.getSpeedPerDegree());
//
////            telemetry.addData("Left Dist", "%.3f  %.3f  %.3f", calibLeft.getBreakPointLeft(), calibLeft.getBreakPointRight(), calibLeft.getBreakPointHor());
////            telemetry.addData("Right Dist", "%.3f  %.3f  %.3f", calibRight.getBreakPointLeft(), calibRight.getBreakPointRight(), calibRight.getBreakPointHor());
////
//
//            saveConfigDiag(mrDiag);
//            showMotorReduction( mrDiag);
//            telemetry.update();
//
//        //max angle left/right
//        //speed per degree left/right
//        //speed adjustment
//            //when to break
//        }
//        catch (Exception ex){
//            telemetry.addData("Error", ex.getMessage());
//            telemetry.update();
//        }
//    }
//
//    private void diagMR(boolean left, MotorReductionCalib calib)
//    {
//        while (!calib.isCalibComplete()) {
//            double distanceInches = desiredX;
//
//
//            double leftOdoStart = bot.getLeftOdemeter();
//            double rightOdoStart = bot.getRightOdemeter();
//            double horOdoStart = bot.getHorizontalOdemeter();
//
//            double startHead = bot.getGyroHeading();
//
////        double distance = Math.abs(distanceInches * bot.COUNTS_PER_INCH_REV);
////        double horDistance = distance * Math.sin(Math.toRadians(desiredAngle));
////        double verDistance = distance * Math.cos(Math.toRadians(desiredAngle));
////        calib.setHorOdoDistance(horDistance);
////        calib.setLeftOdoDistance(verDistance);
////        calib.setRightOdoDistance(verDistance);
//
//            bot.diagToCalib(calib.getCalibSpeed(), 0, distanceInches, left, calib);
//
//
//            timer.reset();
//            while (timer.milliseconds() < 2000 && opModeIsActive()) {
////                    telemetry.addData("Gyroscope", "Stabilizing ...");
////                    telemetry.update();
//            }
//
//            double leftOdoEnd = bot.getLeftOdemeter();
//            double rightOdoEnd = bot.getRightOdemeter();
//
//            double finalHead = bot.getGyroHeading();
//
//            double leftDistanceActual = Math.abs(leftOdoEnd - leftOdoStart);
//            double rightDistanceActual = Math.abs(rightOdoEnd - rightOdoStart);
//
//            double headChange = Math.abs(finalHead - startHead);
//            double reduction = 1;
//            if (headChange >= 2) {
//                this.led.needAdjustment();
//
//                if (leftDistanceActual > rightDistanceActual) {
//                    reduction = rightDistanceActual / leftDistanceActual;
//                    if (left) {
//                        calib.setMotorName(MotorName.LB);
//                    } else {
//                        calib.setMotorName(MotorName.LF);
//                    }
//                } else {
//                    reduction = leftDistanceActual / rightDistanceActual;
//                    if (left) {
//                        calib.setMotorName(MotorName.RF);
//                    } else {
//                        calib.setMotorName(MotorName.RB);
//                    }
//                }
//                calib.setMotorReduction(reduction);
//                calib.setHeadChange(headChange);
//                restoreHead();
//            } else {
//                calib.setCalibComplete(true);
//                this.led.OK();
//            }
//
//
//            timer.reset();
//            while (timer.milliseconds() < 2000 && opModeIsActive()) {
////                    telemetry.addData("Angle", angle);
////                    telemetry.addData("startHead", startHead);
////                    telemetry.addData("finalHead", finalHead);
////                    telemetry.addData("Diag", "About to go back ...");
////                    telemetry.update();
//            }
//
//            //go back
//            bot.diagToCalib(calib.getCalibSpeed(), 0, -distanceInches, left, null);
//
//            timer.reset();
//            while (timer.milliseconds() < 1000 && opModeIsActive()) {
////                    telemetry.addData("Gyroscope", "Stabilizing ...");
////                    telemetry.update();
//            }
//
//            restoreHead();
//            this.led.none();
//        }
//
//        calib.computeSpeedReduction();
//
//        timer.reset();
//        while (timer.milliseconds() < 2000 && opModeIsActive()) {
//
//        }
//
//    }

    private DiagCalibConfig diagAngle(boolean left, MotorReductionBot calib)
    {
        DiagCalibConfig diagConfig = new DiagCalibConfig();
        double [] speeds = new double[]{0.05, 0.1};
        for (int i = 0; i < speeds.length; i++) {
            double distanceInches = desiredX;


            double leftOdoStart = bot.getLeftOdemeter();
            double rightOdoStart = bot.getRightOdemeter();
            double horOdoStart = bot.getHorizontalOdemeter();

            double startHead = bot.getGyroHeading();

//        double distance = Math.abs(distanceInches * bot.COUNTS_PER_INCH_REV);
//        double horDistance = distance * Math.sin(Math.toRadians(desiredAngle));
//        double verDistance = distance * Math.cos(Math.toRadians(desiredAngle));
//        calib.setHorOdoDistance(horDistance);
//        calib.setLeftOdoDistance(verDistance);
//        calib.setRightOdoDistance(verDistance);

            bot.diagToCalib(desiredSpeed, speeds[i], distanceInches, left, calib);


            timer.reset();
            while (timer.milliseconds() < 2000 && opModeIsActive()) {
//                    telemetry.addData("Gyroscope", "Stabilizing ...");
//                    telemetry.update();
            }

            double leftOdoEnd = bot.getLeftOdemeter();
            double rightOdoEnd = bot.getRightOdemeter();
            double horOdoEnd = bot.getHorizontalOdemeter();

            double finalHead = bot.getGyroHeading();

            double leftDistanceActual = Math.abs(leftOdoEnd - leftOdoStart);
            double rightDistanceActual = Math.abs(rightOdoEnd - rightOdoStart);
            double horDistanceActual = Math.abs(horOdoEnd - horOdoStart);

            double headChange = Math.abs(finalHead - startHead);


//        calib.setLeftOdoDistanceActual(leftDistanceActual);
//        calib.setRightOdoDistanceActual(rightDistanceActual);
//        calib.setHorOdoDistanceActual(horDistanceActual);

            double averageVerticalDistance = (leftDistanceActual + rightDistanceActual) / 2;

            double actualAngle = Math.toDegrees(Math.atan(horDistanceActual / averageVerticalDistance));
            diagConfig.setSpeedDegreeData(speeds[i], actualAngle);

            timer.reset();
            while (timer.milliseconds() < 2000 && opModeIsActive()) {
//                    telemetry.addData("Angle", angle);
//                    telemetry.addData("startHead", startHead);
//                    telemetry.addData("finalHead", finalHead);
//                    telemetry.addData("Diag", "About to go back ...");
//                    telemetry.update();
            }

            //go back
            bot.diagToCalib(desiredSpeed, speeds[i], -distanceInches, left, null);

            timer.reset();
            while (timer.milliseconds() < 1000 && opModeIsActive()) {
//                    telemetry.addData("Gyroscope", "Stabilizing ...");
//                    telemetry.update();
            }

            restoreHead();
            this.led.none();
        }

        timer.reset();
        while (timer.milliseconds() < 2000 && opModeIsActive()) {

        }

        return diagConfig;
    }

    private void restoreHead(){
        this.bot.spinH(0, 0.1);
    }

    private void saveConfigMove(MotorReductionBot mrForward, MotorReductionBot mrBack){
        BotCalibConfig config = bot.getCalibConfig();
        if (config == null){
            config = new BotCalibConfig();
        }
        MotorReductionBot originalForward = config.getMoveMRForward();

        MotorReductionBot originalBack = config.getMoveMRBack();


        if (mrForward.compare(originalForward)){
            config.setMoveMRForward(mrForward);
        }

        if (mrBack.compare(originalBack)) {
            config.setMoveMRBack(mrBack);
        }

        ReadWriteFile.writeFile(bot.getCalibConfigFile(), config.serialize());
    }

    private void saveConfigBreak(MotorReductionBot mrForward, MotorReductionBot mrBack){
        BotCalibConfig config = bot.getCalibConfig();
        if (config == null){
            config = new BotCalibConfig();
        }
        MotorReductionBot originalForward = config.getMoveMRForward();

        MotorReductionBot originalBack = config.getMoveMRBack();


        originalForward.updateBreakSamples(mrForward);
        originalBack.updateBreakSamples(mrBack);

        ReadWriteFile.writeFile(bot.getCalibConfigFile(), config.serialize());
    }

    private void saveConfigStrafe(MotorReductionBot mrLeft, MotorReductionBot mrRight){
        BotCalibConfig config = bot.getCalibConfig();
        if (config == null){
            config = new BotCalibConfig();
        }

        MotorReductionBot originalLeft = config.getStrafeLeftReduction();

        MotorReductionBot originalRight = config.getStrafeRightReduction();

        boolean changed = false;
        if (mrLeft.compare(originalLeft)){
            config.setStrafeLeftReduction(mrLeft);
            changed = true;
        }

        if (mrRight.compare(originalRight)) {
            config.setStrafeRightReduction(mrRight);
            changed = true;
        }

        if (changed) {
            ReadWriteFile.writeFile(bot.getCalibConfigFile(), config.serialize());
        }

    }

    private void saveConfigDiag(MotorReductionBot mrLeft, MotorReductionBot mrRight){
        BotCalibConfig config = bot.getCalibConfig();
        if (config == null){
            config = new BotCalibConfig();
        }

        MotorReductionBot originalLeft = config.getDiagMRLeft();

        MotorReductionBot originalRight = config.getDiagMRRight();

        boolean changed = false;
        if (mrLeft.compare(originalLeft)){
            config.setDiagMRLeft(mrLeft);
            changed = true;
        }

        if (mrRight.compare(originalRight)) {
            config.setDiagMRRight(mrRight);
            changed = true;
        }

        if (changed) {
            ReadWriteFile.writeFile(bot.getCalibConfigFile(), config.serialize());
        }
    }

    private void showMotorReduction(MotorReductionBot mr){
        telemetry.addData("*  ","%.2f ||--  --|| %.2f", mr.getLF(), mr.getRF());
        telemetry.addData("*  ", "             ||");
        telemetry.addData("*  ", "             ||");
        telemetry.addData("*  ","%.2f ||--  --|| %.2f", mr.getLB(), mr.getRB());
    }

    private void showMotorReductionCalib(MotorReductionBotCalib mr){
        String veer = " | ";
        if (mr.getVeer() == RobotVeer.LEFT){
            veer = "<--";
        }
        if (mr.getVeer() == RobotVeer.RIGHT){
            veer = "-->";
        }
        telemetry.addData("Calib", mr.getDirection().name());
        telemetry.addData("*  ", "%s: %.2f Dist; %.2f ", veer, mr.getHeadChange(), mr.getDistanceRatio());
        telemetry.addData("*  ", "L:%.2f R: %.2f ", mr.getOverDriveLeft()/bot.COUNTS_PER_INCH_REV, mr.getOverDriveRight()/bot.COUNTS_PER_INCH_REV);
        telemetry.addData("*  ", "--------------------");
        telemetry.addData("*  ","%.2f%s||--  --||%s%.2f", mr.getLF(), mr.getSelectedIndicator(0), mr.getSelectedIndicator(1), mr.getRF());
        telemetry.addData("*  ", "             ||");
        telemetry.addData("*  ", "             ||");
        telemetry.addData("*  ","%.2f%s||--  --||%s%.2f", mr.getLB(), mr.getSelectedIndicator(3), mr.getSelectedIndicator(2), mr.getRB());
        telemetry.addData("*  ", "--------------------");
        telemetry.addData("   ", "Stats:              ");
        telemetry.addData("   ", "Speed: Top %.2f, Average: %.2f", mr.getStats().getFullSpeed(), mr.getStats().getAverageSpeed());
        telemetry.addData("   ", "Dist: T: %.2f, A: %.2f, S: %.2f, Delta: %.2f", mr.getStats().getTotalDistance(), mr.getStats().getAccelerateDistance(), mr.getStats().getSlowDownDistance(), mr.getStats().getSlowDownDelta());
        telemetry.addData("   ", "Time: T: %.2f, A: %.2f, S: %.2f", mr.getStats().getTotalTime(), mr.getStats().getAccelerateTime(), mr.getStats().getSlowDownTime());
    }

}
