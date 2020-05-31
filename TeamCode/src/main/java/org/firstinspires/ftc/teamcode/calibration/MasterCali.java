package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.bots.YellowBot;
import org.firstinspires.ftc.teamcode.gamefield.FieldStats;
import org.firstinspires.ftc.teamcode.skills.Led;

import java.util.concurrent.TimeUnit;


@TeleOp(name="MasterCali", group="Robot15173")
public class MasterCali extends LinearOpMode {

    private YellowBot bot = new YellowBot();
    ElapsedTime timer = new ElapsedTime();


    private MotorReductionList leftReductionStrafe = new MotorReductionList();
    private MotorReductionList rightReductionStrafe = new MotorReductionList();

    private MotorReductionCalib currentMRLeft = new MotorReductionCalib(MotorName.NONE);
    private MotorReductionCalib currentMRRight = new MotorReductionCalib(MotorName.NONE);

    boolean strafeDirLeft = true;

    private static double CALIB_SPEED = 0.5;
    private static double CALIB_SPEED_HIGH = 0.8;
    private static double CALIB_SPEED_LOW = 0.2;

    private double rightLong = 0;
    private double rightPerDegree = 0;

    private double leftLong = 0;
    private double leftPerDegree = 0;

    private double separation = 0;
    private double horizontalTicksDegreeLeft = 0;
    private double horizontalTicksDegreeRight = 0;

    private double actualAngle = 0;


    Deadline gamepadRateLimit;
    private final static int GAMEPAD_LOCKOUT = 500;

    private static final int[] modes = new int[]{0, 1, 2, 3};
    private static final String[] modeNames = new String[]{"Straight", "Spin", "Strafe", "Diag"};

    private int selectedMode = 0;

    private boolean speedSettingMode = false;
    private boolean angleSettingMode = false;

    private int desiredAngle = 45;
    private static int DIAG = 45;
    private double desiredSpeed = CALIB_SPEED;

    private Led led = null;


    @Override
    public void runOpMode() throws InterruptedException {
        try {
            bot.init(this, hardwareMap, telemetry);
            bot.initGyro();
            bot.initCalibData();

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
            angleSettingMode = !angleSettingMode;
            gamepadRateLimit.reset();
            showStatus();
        }


        if (gamepad1.dpad_down){
            if (speedSettingMode){
                if (desiredSpeed > 0){
                    desiredSpeed = desiredSpeed - 0.1;
                }
            }
            else if (angleSettingMode){
                if (desiredAngle > 0){
                    desiredAngle = desiredAngle - 5;
                }
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
            else if (angleSettingMode){
                if (desiredAngle < 90){
                    desiredAngle = desiredAngle + 5;
                }
            }
            else {
                if (selectedMode > 0) {
                    selectedMode--;
                }
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
//                    calibStrafe();
                    break;
                case 3:
                    calibDiag();
                    break;

            }
        }
    }

    private void showStatus(){

        if (speedSettingMode){
            telemetry.addData("Speed", desiredSpeed);
        }
        else if (angleSettingMode){
            telemetry.addData("Angle", desiredAngle);
        }
        else {
            for (int i = 0; i < modes.length; i++) {
                telemetry.addData(modeNames[i], i == selectedMode);
            }
        }

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

        horizontalTicksDegreeLeft = Math.abs(horizontalShift/actualAngle);

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("SPin","Will go to right now");
            telemetry.update();
        }

        spinRight();



        //separation
        separation = 2*90 * ((leftLong - rightLong)/actualAngle)/(Math.PI*bot.COUNTS_PER_INCH_REV);

        BotCalibConfig config = bot.getCalibConfig();
        if (config == null){
            config = new BotCalibConfig();
        }

        config.setLeftTickPerDegree(Math.abs(leftPerDegree));
        config.setRightTickPerDegree(Math.abs(rightPerDegree));
        config.setWheelBaseSeparation(Math.abs(separation));
        config.setHorizontalTicksDegree(horizontalTicksDegreeLeft);
        ReadWriteFile.writeFile(bot.getCalibConfigFile(), config.serialize());

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("Spin","Finlizing....");
            telemetry.update();
        }

        restoreHead();

        telemetry.addData("Spin","Calibration complete");
        telemetry.addData("leftLong", leftLong);
        telemetry.addData("leftPerDegree", leftPerDegree);

        telemetry.addData("rightLong", rightLong);
        telemetry.addData("rightPerDegree", rightPerDegree);

        telemetry.addData("separation", separation);
        telemetry.addData("horizontalTicksDegree Left", horizontalTicksDegreeLeft);
        telemetry.addData("horizontalTicksDegree Right", horizontalTicksDegreeRight);
        telemetry.addData("actualAngle", actualAngle);

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
        actualAngle = finalHead - currentHead;


        double horizontalPosition = bot.getHorizontalOdemeter();
        double horizontalShift = horizontalPosition - horizontalStart;

        horizontalTicksDegreeRight = Math.abs(horizontalShift/actualAngle);
    }

    private void calibStrafe(){
        double headChange = 0;

        boolean stop = false;
        while(!stop) {
            headChange = strafe();
            if (currentMRRight.isCalibComplete() && currentMRLeft.isCalibComplete()){
                break;
            }
        }
        telemetry.addData("Strafe", "Calibration complete");
        telemetry.addData("Last turnDegrees", headChange);
        telemetry.addData("Left Reduction", leftReductionStrafe.toString());
        telemetry.addData("strafeRightReduction", rightReductionStrafe.toString());
        telemetry.update();
        saveConfigStrafe();
    }

    private double strafe(){
        double headChange = 0;
        double currentHead = bot.getGyroHeading();


        bot.strafeTo(CALIB_SPEED, 30, strafeDirLeft, leftReductionStrafe, rightReductionStrafe);

        MotorReductionList currentList = rightReductionStrafe;
        MotorReductionCalib currentMRCalib = currentMRRight;
        if (strafeDirLeft){
            currentList = leftReductionStrafe;
            currentMRCalib = currentMRLeft;
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
        if (currentMRCalib.isCalibComplete()){
            //do nothing
        }
        else {
            //set values if fully calibrated
            if (headChange >= -2 && headChange <= 2) {
                currentMRCalib.setCalibComplete(true);
            } else {
                //first time
                if (currentMRCalib.getMotorName() == MotorName.NONE){
                    currentMRCalib.setMotorName(currentList.getFirsttMR().getMotorName());
                    currentMRCalib.setOriginalHeadChange(headChange);
                    currentMRCalib.setHeadChange(headChange);
                    currentMRCalib.setReductionStep(MotorReductionCalib.DEFAULT_REDUCTION_STEP);
                }
                else{
                    //compare current change in heading with the previous
                    double oldHeadChange = currentMRCalib.getHeadChange();
                    boolean not_improved = (oldHeadChange < 0 && headChange < oldHeadChange) || (oldHeadChange > 0 && headChange > oldHeadChange);
                    if (not_improved){
                        //try another motor next time
                        currentList.restoreList();
                        MotorReduction next = currentList.getNextMR(currentMRCalib.getMotorName());
                        currentMRCalib.setMotorName(next.getMotorName());
                        currentMRCalib.setReductionStep(MotorReductionCalib.DEFAULT_REDUCTION_STEP);
                        currentMRCalib.setHeadChange(currentMRCalib.getOriginalHeadChange());
                    }
                    else{
                        double diff = Math.abs(oldHeadChange - headChange);
                        if (diff < Math.abs(oldHeadChange)){
                            //need to reduce more
                            double adjustment = Math.abs(headChange)/diff * currentMRCalib.getReductionStep();
                            currentMRCalib.setReductionStep(adjustment);
                        }
                        else{
                            //overkill. need to reduce less
                            double adjustment = Math.abs(headChange)/diff * currentMRCalib.getReductionStep();
                            currentMRCalib.setReductionStep(-adjustment);
                        }
                    }
                }
                currentMRCalib.adjustReduction();
                currentList.updateList(currentMRCalib.getMR());
            }
        }

        timer.reset();
        while(timer.milliseconds() < 3000 && opModeIsActive()){
            telemetry.addData("Cycle","Complete");
            telemetry.addData("strafeDirLeft", strafeDirLeft);
            telemetry.addData("headChange", headChange);
            telemetry.addData("startHead", currentHead);
            telemetry.addData("finalHead", finalHead);
            telemetry.addData("Motor", currentMRCalib.getMotorName());
            telemetry.addData("Reduction step", currentMRCalib.getReductionStep());
            telemetry.addData("strafeReduction", currentMRCalib.getMotorReduction());
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

    private void calibDiag(){
        try {
            led.none();
            MotorReductionCalib calibLeft = new MotorReductionCalib();
            calibLeft.setCalibSpeed(desiredSpeed);

            DiagCalibConfig dcLeft = null;
            DiagCalibConfig dcRight = null;

            while (!calibLeft.isCalibComplete()){
                dcLeft =  diag(true, calibLeft);
            }


            MotorReductionCalib calibRight = new MotorReductionCalib();
            calibRight.setCalibSpeed(desiredSpeed);
            while (!calibRight.isCalibComplete()){
                dcRight = diag(false, calibRight);
            }


            double leftF = 1;
            double leftB = 1;

            double rightF = 1;
            double rightB = 1;

            if (calibLeft.getMotorName() == MotorName.LB){
                leftB = calibLeft.getMotorReduction();
            } else if (calibLeft.getMotorName() == MotorName.RF){
                rightF = calibLeft.getMotorReduction();
            }


            if (calibRight.getMotorName() == MotorName.LF){
                leftF = calibRight.getMotorReduction();
            } else if (calibRight.getMotorName() == MotorName.RB){
                rightB = calibRight.getMotorReduction();
            }

            dcLeft.computeSpeedPerDegree();
            dcRight.computeSpeedPerDegree();

            telemetry.addData("Speed", desiredSpeed);

            telemetry.addData("Left Angle", dcLeft.getMaxAgle());
            telemetry.addData("Right Agle", dcRight.getMaxAgle());

            telemetry.addData("Left Speed/degree", dcLeft.getSpeedPerDegree());
            telemetry.addData("Right Speed/degree", dcRight.getSpeedPerDegree());

//            telemetry.addData("Left Dist", "%.3f  %.3f  %.3f", calibLeft.getBreakPointLeft(), calibLeft.getBreakPointRight(), calibLeft.getBreakPointHor());
//            telemetry.addData("Right Dist", "%.3f  %.3f  %.3f", calibRight.getBreakPointLeft(), calibRight.getBreakPointRight(), calibRight.getBreakPointHor());
//

            showMotorReduction( leftF,  rightF,  leftB,  rightB);
            telemetry.update();

        //max angle left/right
        //speed per degree left/right
        //speed adjustment
            //when to break
        }
        catch (Exception ex){
            telemetry.addData("Error", ex.getMessage());
            telemetry.update();
        }
    }

    private DiagCalibConfig diag(boolean left, MotorReductionCalib calib)
    {
        DiagCalibConfig diagConfig = new DiagCalibConfig();
        double [] speeds = new double[]{0, 0.05, 0.1};
        for (int i = 0; i < speeds.length; i++) {
            double distanceInches = 30;


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

            bot.diagToCalib(calib.getCalibSpeed(), speeds[i], distanceInches, left, calib);


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
            double reduction = 1;
            if (headChange >= 2) {
                this.led.needAdjustment();

                if (leftDistanceActual > rightDistanceActual) {
                    reduction = rightDistanceActual / leftDistanceActual;
                    if (left) {
                        calib.setMotorName(MotorName.LB);
                    } else {
                        calib.setMotorName(MotorName.LF);
                    }
                } else {
                    reduction = leftDistanceActual / rightDistanceActual;
                    if (left) {
                        calib.setMotorName(MotorName.RF);
                    } else {
                        calib.setMotorName(MotorName.RB);
                    }
                }
                calib.setHeadChange(headChange);
                restoreHead();
            } else {
                this.led.OK();
            }

//        calib.setLeftOdoDistanceActual(leftDistanceActual);
//        calib.setRightOdoDistanceActual(rightDistanceActual);
//        calib.setHorOdoDistanceActual(horDistanceActual);

            double averageVerticalDistance = (leftDistanceActual + rightDistanceActual) / 2;

            double actualAngle = Math.toDegrees(Math.atan(horDistanceActual / averageVerticalDistance));
            diagConfig.setSpeedDegreeData(speeds[i], actualAngle);
            if (i == 0) {
                calib.setMotorReduction(reduction);
            }


            timer.reset();
            while (timer.milliseconds() < 2000 && opModeIsActive()) {
//                    telemetry.addData("Angle", angle);
//                    telemetry.addData("startHead", startHead);
//                    telemetry.addData("finalHead", finalHead);
//                    telemetry.addData("Diag", "About to go back ...");
//                    telemetry.update();
            }

            //go back
            bot.diagToCalib(calib.getCalibSpeed(), speeds[i], -distanceInches, left, null);

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

        calib.computeSpeedReduction();
        calib.setCalibComplete(true);

        return diagConfig;
    }

    private void restoreHead(){
        this.bot.spinH(0, 0.1);
    }

    private void saveConfigStrafe(){
    BotCalibConfig config = bot.getCalibConfig();
        if (config == null){
            config = new BotCalibConfig();
        }

//        config.setStrafeLeftReduction(strafeLeftReduction);
//        config.setStrafeRightReduction(strafeRightReduction);
        ReadWriteFile.writeFile(bot.getCalibConfigFile(), config.serialize());
    }

    private void showMotorReduction(double leftF, double rightF, double leftB, double rightB){
        telemetry.addData("*  ","%.2f ||--  --|| %.2f", leftF, rightF);
        telemetry.addData("*  ", "             ||");
        telemetry.addData("*  ", "             ||");
        telemetry.addData("*  ","%.2f ||--  --|| %.2f", leftB, rightB);
    }

}
