package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.bots.SimpleBot;
import org.firstinspires.ftc.teamcode.skills.DetectionInterface;
import org.firstinspires.ftc.teamcode.skills.StoneFinder;

import java.util.List;


/**
 * Created by sjeltuhin on 1/15/18.
 */

public abstract class AutoBase extends LinearOpMode {

    protected SimpleBot robot = new SimpleBot();   // Use our standard robot configuration
    protected ElapsedTime runtime = new ElapsedTime();
    protected boolean stoneDetected = false;
    protected float stoneLeft = -1;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

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

                /**
                 * Activate TensorFlow Object Detection before we wait for the start command.
                 * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
                 **/
                if (tfod != null) {
                    tfod.activate();
                }
                telemetry.addData("Info", "TF Activated");
            }
            catch (Exception ex){
                telemetry.addData("Error", "Unable to initialize Tensor Flow");
            }
        }
        telemetry.update();

        try {
            waitForStart();
            act();
        }
        catch (Exception ex){
            telemetry.addData("Issues autonomous initialization", ex);
        }

        telemetry.update();

    }

    protected void initRobot(){
        try{
        robot.init(hardwareMap, telemetry);
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
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    protected void act(){

    }

    protected boolean detectStone(int timeout){
        StoneFinder sf = new StoneFinder(tfod);
        boolean found = sf.detectStone(timeout, telemetry);
        if (found){
            stoneLeft = sf.getStoneLeft();
        }
        return found;
    }

    protected void stopStoneDetection(){
        if (tfod != null) {
            tfod.shutdown();
        }
    }


    protected void move(double speed, double moveTo){
        telemetry.addData("Auto", "Distance = %.2f", moveTo);
        telemetry.update();
        robot.encoderDrive(speed, moveTo, moveTo,0, telemetry);

        robot.stop();
    }

    protected void moveDetect(double speed, double moveTo){
        final StoneFinder sf = new StoneFinder(tfod);
        robot.encoderMoveDetect(speed, moveTo, moveTo, 0, telemetry, new DetectionInterface() {
            @Override
            public boolean detect() {
                if (!stoneDetected) {
                    stoneDetected = sf.detect(telemetry);
                }
                return stoneDetected;
            }
        });

        robot.stop();
    }


    protected void moveUntil(double speed, int moveUntil, int max){
        robot.move(-speed, 0, telemetry);
        int pos = robot.rightDriveFront.getCurrentPosition();
        int target = pos + robot.getDriveIncrement(max);
        while (true){
            double range = robot.getRangetoObstacle();
            telemetry.addData("Range", range);
            telemetry.update();
            int current = robot.rightDriveFront.getCurrentPosition();
            if (range > 0 && (range < moveUntil || Math.abs(current) > Math.abs(target))){
                break;
            }
        }
        robot.stop();
    }

    protected void strafe(double speed, double moveTo){
        telemetry.addData("Auto", "Distance = %.2f", moveTo);
        telemetry.update();
        robot.encoderStrafe(speed, moveTo, 0, telemetry);
        robot.stop();
    }

    protected void strafeRight(double speed, double moveTo){
        int increment = robot.getStrafeIncrement(moveTo);
        telemetry.addData("Strafe", "Increment = %d", increment);
        int pos = robot.leftDriveBack.getCurrentPosition();
        telemetry.addData("Strafe", "Start pos = %d", pos);
        int current = pos;
        robot.strafeLeft(speed, telemetry);
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
        robot.strafeRight(speed, telemetry);
        while (current > pos - increment){
            current = robot.leftDriveBack.getCurrentPosition();
            telemetry.addData("Strafe", "Current pos = %d", current);
            telemetry.update();
        }
        robot.stop();
    }

    protected void turn(double speed, double moveTo){
        telemetry.addData("Auto", "Distance = %.2f", moveTo);
        telemetry.update();
        robot.encoderTurn(speed, moveTo, moveTo, 0, telemetry);
        robot.stop();
    }

    protected void unfoldIntake(){
        robot.encoderIntakeMove(-1, robot.INTAKE_PIVOT_SPEED, 1, telemetry);
    }

    protected void foldIntake(){
        robot.encoderIntakeMove(1, robot.INTAKE_PIVOT_SPEED_UP, 1, telemetry);
//        robot.intakePivot.setPower(-robot.ANTI_GRAVITY_POWER);
    }


}
