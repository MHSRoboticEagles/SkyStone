package org.firstinspires.ftc.teamcode.skills;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


public class StoneFinder {
    Telemetry telemetry;
    private TFObjectDetector tfod;
    private HardwareMap hardwareMap;
    private float stoneLeft = -1;
    private float stoneWidth = -1;
    private float stoneTop = -1;
    private double angle = 0;
    private double distanceToObject = -1;
    private double realWidth = 8;
    private double focalength = 999;
    private static final String LABEL_SKYSTONE = "Skystone";
    private static final double CAMERA_HEIGHT = 11;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";

    private static final String VUFORIA_KEY =
            "AZs0syj/////AAABmaxIME6H4k74lx12Yv3gnoYvtGHACOflWi3Ej36sE7Hn86xDafDA3vhzxSOgBtyNIQ1ua6KP2j3I2ScFedVw8n6MJ7PReZQP4sTdc8gHvoy17hD574exMmoUQ3rVUMkgU2fwN2enw2X+Ls2F3BLuCg/A4SBZjzG3cweO+owiKO/2iSIpeC4rBdUnPiTxqPHNa8UOxyncCGV0+ZFXresQm/rK7HbOKB9MEszi8eW2JNyfjTdKozwDxikeDRV7yPvoIhZ5A+mrrC1GgrEzYwNTVHeki2cg4Ea62pYwdscaJ+6IWHlBIDutqmgJu/Os3kAzZkOh0TJ8P3e29Ou4ZczTdDH0oqkPt78Nt4VdYbSbLRCw";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;



    public StoneFinder(TFObjectDetector tf) {
        tfod = tf;
    }

    public StoneFinder(HardwareMap hMap, Telemetry t) {
        hardwareMap = hMap;
        telemetry = t;
        initVuforia();
    }

    public boolean detectStone(int timeout, Telemetry telemetry, LinearOpMode caller){
        boolean found = false;

        boolean stop = false;
        ElapsedTime runtime = new ElapsedTime();
        while (!stop && runtime.seconds() < timeout) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        telemetry.addData(String.format("  Width/height (%d)", i), "%.03f , %.03f, %d , %d",
                                recognition.getWidth(), recognition.getHeight(), recognition.getImageWidth(), recognition.getImageHeight());
                        if (recognition.getLabel().contentEquals(LABEL_SKYSTONE)) {
                            found = true;
                            telemetry.addData("Found", found);
                            stoneLeft = recognition.getLeft();
                            stoneWidth = recognition.getWidth();
                            stoneTop = recognition.getTop();
                            angle = recognition.estimateAngleToObject(AngleUnit.RADIANS);
                            setDistanceToObject(focalength * realWidth / stoneWidth);
                            break;
                        }
                    }
                    telemetry.update();
                }
            }
            if (found || !caller.opModeIsActive()){
                stop = true;
            }
        }

        return found;
    }


    public float getStoneLeft() {
        return stoneLeft;
    }
    public float getStoneCenter() {
        return stoneLeft + stoneWidth/2;
    }
    public float getStoneWidth() {
        return stoneWidth;
    }

    public float getStoneTop() {
        return stoneTop;
    }

    public double getAngle() {
        return angle;
    }

    public double getDistanceToObject() {
        return distanceToObject;
    }

    public void setDistanceToObject(double distanceToObject) {
        this.distanceToObject = distanceToObject;
    }

    public void initRec(){
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
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SKYSTONE);
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

    public void stopStoneDetection(){
        if (tfod != null) {
            tfod.shutdown();
        }
    }

}
