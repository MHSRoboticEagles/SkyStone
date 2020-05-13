package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.bots.TieBot;
import org.firstinspires.ftc.teamcode.skills.DetectionInterface;
import org.firstinspires.ftc.teamcode.skills.StoneFinder;


/**
 * Created by sjeltuhin on 1/15/18.
 */

public abstract class AutoBase extends LinearOpMode {

    protected TieBot robot = new TieBot();   // Use our standard robot configuration
    protected ElapsedTime runtime = new ElapsedTime();
    protected boolean stoneDetected = false;
    protected boolean stoneInside = false;
    protected float stoneLeft = -1;
    protected float stoneWidth = -1;
    protected float stoneTop = -1;
    protected double stoneAngle = -999;
    protected double stoneDistance = -1;
//    protected  ColorCheck colorChecker = null;

    protected int skyStoneIndex = 0; // 1-based

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SKYSTONE = "Skystone";

    private static final String VUFORIA_KEY =
            "AZs0syj/////AAABmaxIME6H4k74lx12Yv3gnoYvtGHACOflWi3Ej36sE7Hn86xDafDA3vhzxSOgBtyNIQ1ua6KP2j3I2ScFedVw8n6MJ7PReZQP4sTdc8gHvoy17hD574exMmoUQ3rVUMkgU2fwN2enw2X+Ls2F3BLuCg/A4SBZjzG3cweO+owiKO/2iSIpeC4rBdUnPiTxqPHNa8UOxyncCGV0+ZFXresQm/rK7HbOKB9MEszi8eW2JNyfjTdKozwDxikeDRV7yPvoIhZ5A+mrrC1GgrEzYwNTVHeki2cg4Ea62pYwdscaJ+6IWHlBIDutqmgJu/Os3kAzZkOh0TJ8P3e29Ou4ZczTdDH0oqkPt78Nt4VdYbSbLRCw";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;



    protected void runAutoMode(){
        initRobot();
        initVuforia();
        try {
            preStart();
            waitForStart();
            act();
        }
        catch (Exception ex){
            telemetry.addData("Issues autonomous initialization", ex);
        }

        telemetry.update();

    }

    protected void initRec(){
        if (vuforia == null){
            telemetry.addData("Error", "Unable to start Vuforia");
        }
        else {
            telemetry.addData("Info", "Vuforia initialized");

            try {

                if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                    initTfod();
                    telemetry.addData("Info", "TF initialized");
                } else {
                    telemetry.addData("Error", "This device is not compatible with TFOD");
                }

                activateTfod();
            }
            catch (Exception ex){
                telemetry.addData("Error", "Unable to initialize Tensor Flow");
            }
        }
        telemetry.update();
    }

    protected void activateTfod(){
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }
        telemetry.addData("Info", "TF Activated");
    }

    protected void preStart(){

    }

    protected void initRobot(){
        try{
            robot.init(hardwareMap, telemetry);
            robot.startGyro(hardwareMap, telemetry);;
            robot.initSensors();
        }
        catch (Exception ex){
            telemetry.addData("Init", ex.getMessage());
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.7;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SKYSTONE);
    }

    protected void act(){

    }


    protected boolean detectStone(int timeout){
        StoneFinder sf = new StoneFinder(tfod);
        boolean found = sf.detectStone(timeout, telemetry, this);
        if (found){
            stoneLeft = sf.getStoneLeft();
            stoneWidth = sf.getStoneWidth();
            stoneTop = sf.getStoneTop();
        }
        return found;
    }

    protected boolean detectStoneMove(double power, double moveTo){
        robot.encoderStartMove(power, moveTo, moveTo, 0, this);
        StoneFinder sf = new StoneFinder(tfod);
        boolean found = false;
        while (robot.motorsBusy()) {
            found = sf.detectStoneContinous(telemetry, this);
            if (found) {
                stoneAngle = sf.getAngle();
                stoneDistance = sf.getDistanceToObject();
                break;
            }
        }
        robot.encoderStopMove();
        return found;
    }


    protected StoneFinder getStoneLocation(){
        StoneFinder sf = new StoneFinder(tfod);
        boolean found = sf.detectStoneContinous(telemetry, this);
        if (found){
            stoneLeft = sf.getStoneLeft();
            stoneWidth = sf.getStoneWidth();
            stoneTop = sf.getStoneTop();
            return sf;
        }
        return  null;

    }

    protected void stopStoneDetection(){
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    protected double intakeStone(double speed, long until){
        robot.moveIntake(1);

        double moved = robot.encoderMoveDetect(speed, until, until, 0, this, new DetectionInterface() {
            @Override
            public boolean detect() {
                telemetry.addData("Detect", "inside detect");
                if (!stoneInside) {
                    telemetry.addData("Detect", "About to call color");
                    stoneInside = robot.isStoneInside();
                    if (stoneInside){
                        robot.toggleStoneLock(true);
                        robot.moveIntake(0);
                    }
                }
                telemetry.addData("Detect return", stoneInside);
                return stoneInside;
            }
        });
        if (!stoneInside) {
            stoneInside = robot.isStoneInside();
            if (stoneInside) {
                robot.toggleStoneLock(true);
                robot.moveIntake(0);
            }
        }
        return moved;
    }

    protected  void intakeStoneNoSensor(double speed, long until){
        robot.moveIntake(1);

        robot.encoderDrive(speed, until, until, 0, this);
        if (!stoneInside) {
            stoneInside = robot.isStoneInside();
            if (stoneInside) {
                robot.toggleStoneLock(true);
                robot.moveIntake(0);
            }
        }
    }



    protected void move(double speed, double moveTo){
       move(speed, moveTo, 0);
    }

    protected void move(double speed, double moveTo, int timeoutMS){
        telemetry.addData("Auto", "Distance = %.2f", moveTo);
        telemetry.update();
        robot.encoderDrive(speed, moveTo, moveTo, timeoutMS, this);
    }

    protected boolean moveDetect(double speed, double moveTo){
        stoneDetected = false;
        final StoneFinder sf = new StoneFinder(tfod);
        final LinearOpMode caller = this;
        robot.encoderMoveDetect(speed, moveTo, moveTo, 0, this, new DetectionInterface() {
            @Override
            public boolean detect() {
                telemetry.addData("Info", "Calling detect");
                stoneDetected = sf.detectStoneContinous(telemetry, caller);
                telemetry.addData("Found", stoneDetected);
                telemetry.update();
                return stoneDetected;
            }
        });

        robot.stop();
        return stoneDetected;
    }



    protected void moveLeftUntil(double speed, int moveUntil, boolean stop){
        double start = robot.getRangetoObstacleLeft();
        if (start < 0 || (moveUntil <= start + 2  && moveUntil >= start -2)){
            return;
        }
        boolean closer = moveUntil < start;
        if (closer){
            robot.strafeLeft(speed);
        }
        else{
            robot.strafeRight(speed);
        }

        while (true){
            double range = robot.getRangetoObstacleLeft();
            telemetry.addData("rangeLeft", range);
            if (closer) {
                if (range > -1 && (range <= moveUntil )) {
                    break;
                }
            }
            else{
                if (range > -1 && (range >= moveUntil )) {
                    break;
                }
            }
        }
        if (stop) {
            robot.stop();
        }
    }

    protected void moveRightUntil(double speed, int moveUntil, boolean stop){
        double start = robot.getRangetoObstacleRight();
        if (start < 0 || (moveUntil <= start + 2  && moveUntil >= start -2)){
            return;
        }
        boolean closer = moveUntil < start;
        if (closer){
            robot.strafeRight(speed);
        }
        else{
            robot.strafeLeft(speed);
        }


        while (true){
            double range = robot.getRangetoObstacleRight();
            telemetry.addData("rangeRight", range);
            if (closer) {
                if (range > -1 && (range <= moveUntil )) {
                    break;
                }
            }
            else{
                if (range > -1 && (range >= moveUntil )) {
                    break;
                }
            }
        }
        if (stop) {
            robot.stop();
        }
    }

    protected double moveBackUntil(double speed, int moveUntil, int max, boolean stop){
        double start = robot.getRangetoObstacleBack();
        if (start < 0 || (moveUntil <= start + 1  && moveUntil >= start - 1)){
            return 0;
        }
        boolean closer = moveUntil < start;

        if (!closer){
            speed = -speed;
        }
        robot.move(speed, 0);
        int pos = robot.rightDriveFront.getCurrentPosition();
        int target = pos + robot.getDriveIncrement(max);

        while (true){
            double range = robot.getRangetoObstacleBack();
            telemetry.addData("rangeBack", range);
            int current = robot.rightDriveFront.getCurrentPosition();
            if (closer) {
                if (range > -1 && (range <= moveUntil + 0.5 || Math.abs(current) > Math.abs(target))) {
                    break;
                }
            }
            else{
                if (range > -1 && (range >= moveUntil || Math.abs(current) > Math.abs(target))) {
                    break;
                }
            }
        }
        int actual = robot.rightDriveFront.getCurrentPosition();

        double diff = robot.getPositionDiffInches(pos, actual);

        if (stop) {
            robot.stop();
        }

        return diff;
    }


    protected void strafe(double speed, double moveTo){
        telemetry.addData("Auto", "Distance = %.2f", moveTo);
        telemetry.update();
        robot.encoderStrafe(speed, moveTo, 0);
        robot.stop();
    }

    protected void strafeRight(double speed, double moveTo){
        int increment = robot.getStrafeIncrement(moveTo);
        telemetry.addData("Strafe", "Increment = %d", increment);
        int pos = robot.leftDriveBack.getCurrentPosition();
        telemetry.addData("Strafe", "Start pos = %d", pos);
        int current = pos;
        robot.strafeLeft(speed);
        while (current < pos + increment){
            current = robot.leftDriveBack.getCurrentPosition();
            telemetry.addData("Strafe", "Current pos = %d", current);
            telemetry.update();
        }
        robot.stop();
    }

    protected void strafeLeft(double speed, double moveTo){
        int increment = robot.getStrafeIncrement(moveTo);
        telemetry.addData("Strafe", "Increment = %d", increment);
        int pos = robot.leftDriveBack.getCurrentPosition();
        telemetry.addData("Strafe", "Start pos = %d", pos);
        int current = pos;
        robot.strafeRight(speed);
        while (current > pos - increment){
            if (!opModeIsActive()){
                break;
            }
            current = robot.leftDriveBack.getCurrentPosition();
            telemetry.addData("Strafe", "Current pos = %d", current);
            telemetry.update();
        }
        robot.stop();
    }


    protected void strafeRightUntil(double speed, int leftStop, int rightStop){
        robot.strafeLeft(speed);
        int stop = leftStop != 0 ? leftStop : rightStop;
        while (true) {
            double range = leftStop != 0  ? robot.getRangetoObstacleLeft() : robot.getRangetoObstacleRight();
            telemetry.update();
            if (range > 0 && range <= stop ) {
                break;
            }
        }
        robot.stop();
    }

    protected void strafeLeftUntil(double speed, int leftStop, int rightStop){
        robot.strafeRight(speed);
        int stop = leftStop != 0 ? leftStop : rightStop;
        while (true) {
            double range = leftStop != 0  ? robot.getRangetoObstacleLeft() : robot.getRangetoObstacleRight();
            telemetry.update();
            if (range > 0 && range <= stop ) {
                break;
            }
        }
        robot.stop();
    }

    protected void turn(double speed, double moveTo){
        telemetry.addData("Auto", "Distance = %.2f", moveTo);
        telemetry.update();
        robot.encoderTurn(speed, moveTo, moveTo, 0);
        robot.stop();
    }

    protected void releaseStone(){
        this.robot.encoderCrane(1, 8, 5);
        this.robot.swivelStone(true);
        this.robot.encoderCrane(1, 3, 5);
        sleep(2000);
        this.robot.swivelStone(false);
        this.robot.encoderCrane(1, -11, 5);
    }

    public void aimAtStone(double power, double angle, double distance){

        double degrees = angle;

        double head = robot.getGyro().getHeading();
        double targetHead = head;

        double  leftPower = 0, rightPower = 0;


        boolean left = false;
        boolean right = false;
        if (degrees > 0){
            //turn left
            right = true;
            targetHead = head - Math.abs(degrees);
        }
        else if (degrees < 0){
            left = true;
            targetHead = head + Math.abs(degrees);
        }
        else{
            return;
        }


        if (right)
        {   // turn right.
            rightPower = power;
            robot.rightDriveBack.setPower(rightPower);
            robot.rightDriveFront.setPower(rightPower);

            robot.leftDriveBack.setPower(-rightPower);
            robot.leftDriveFront.setPower(-rightPower);
        }
        else if (left)
        {   // turn left.
            leftPower = power;
            robot.leftDriveBack.setPower(leftPower);
            robot.leftDriveFront.setPower(leftPower);

            robot.rightDriveBack.setPower(-leftPower);
            robot.rightDriveFront.setPower(-leftPower);
        }
        else return;


        while (true){
            head = robot.getGyro().getHeading();
            if (!opModeIsActive() || (left && head >= targetHead)
                    || (right && head <= targetHead)){
                break;
            }
        }
        robot.stop();
    }
}
