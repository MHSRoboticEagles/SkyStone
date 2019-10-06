/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.skills;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.gamefield.MineralLineUp;
import org.firstinspires.ftc.teamcode.gamefield.MineralObject;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

public class MineralDetection {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AZs0syj/////AAABmaxIME6H4k74lx12Yv3gnoYvtGHACOflWi3Ej36sE7Hn86xDafDA3vhzxSOgBtyNIQ1ua6KP2j3I2ScFedVw8n6MJ7PReZQP4sTdc8gHvoy17hD574exMmoUQ3rVUMkgU2fwN2enw2X+Ls2F3BLuCg/A4SBZjzG3cweO+owiKO/2iSIpeC4rBdUnPiTxqPHNa8UOxyncCGV0+ZFXresQm/rK7HbOKB9MEszi8eW2JNyfjTdKozwDxikeDRV7yPvoIhZ5A+mrrC1GgrEzYwNTVHeki2cg4Ea62pYwdscaJ+6IWHlBIDutqmgJu/Os3kAzZkOh0TJ8P3e29Ou4ZczTdDH0oqkPt78Nt4VdYbSbLRCw";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private MineralDetectionListener listener = null;
    private MineralLineUp lineUp = null;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    public MineralDetection(VuforiaLocalizer vuforia, HardwareMap hMap, Telemetry t){
        this.vuforia = vuforia;
        this.hardwareMap = hMap;
        this.telemetry = t;
    }

    public void setListener(MineralDetectionListener listener) {
        this.listener = listener;
    }

    public interface MineralDetectionListener {
        boolean onDetection(GoldPosition position);
    }

    public void init() throws Exception {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        try {
//            initVuforia();

            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                initTfod();
            } else {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            }
        }
        catch (Exception ex){
            telemetry.addData("Detection init issues", ex);
            throw new Exception("Detection init issues", ex);
        }
    }

    public void detectGold(){
        detect(0);
    }

    public void detectGold(int timeout){
        detect((timeout));
    }

    protected void detect(int timeout){

        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }

        boolean stop = false;
        ElapsedTime runtime = new ElapsedTime();
        while(!stop || runtime.seconds() < timeout) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                GoldPosition position = GoldPosition.None;
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                position = GoldPosition.Left;
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                position = GoldPosition.Right;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                position = GoldPosition.Center;
                            }
                        }
                    }
                    telemetry.update();
                } else {
                    telemetry.addData("TF", "Nothing detected");
                    telemetry.update();
                }
                if (listener != null) {
                    stop = listener.onDetection(position);
                }
            } else {
                telemetry.addData("TF", "TensorFlow not initialized.");
                telemetry.update();
            }
        }
    }

    /// view determines which 2 elements are in the frame
    /// -1 => indices 0 and 1
    /// 1  => indices 1 and 2
    public MineralLineUp detectFlex(int timeout, int view){

        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }
        else{
            return lineUp;
        }

        boolean stop = false;
        ElapsedTime runtime = new ElapsedTime();
        while(!stop && runtime.seconds() < timeout) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                GoldPosition position = GoldPosition.None;
                List<Recognition> updatedRecognitions = tfod.getRecognitions();

                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());
                    ArrayList<MineralObject> objectsDetected = new ArrayList<>(updatedRecognitions.size());
                    boolean goldFound = false;
                    for (Recognition recognition : updatedRecognitions) {
                        MineralObject obj = new MineralObject(recognition.getLabel(), recognition.getLeft());
                        objectsDetected.add(obj);
                        if (!goldFound){
                            goldFound = MineralLineUp.isGold(obj);
                        }
                    }
                    lineUp = new MineralLineUp(objectsDetected, goldFound, view);
                    lineUp.compute();
                    telemetry.update();
                } else {
                    telemetry.addData("TF", "Nothing detected");
                    telemetry.update();
                }
                //quit if there is 1 gold or 2 silver
                if (lineUp != null && lineUp.isGoldFound() || (updatedRecognitions != null && updatedRecognitions.size() == 2)){
                    stop = true;
                }

//                if (listener != null) {
//                    stop = listener.onDetection(position);
//                }
            } else {
                telemetry.addData("TF", "TensorFlow not initialized.");
                telemetry.update();
            }
        }
        return lineUp;
    }

    public void stopDetection(){
//        telemetry.addData("TF", "Stopping object detection");
//        telemetry.update();

        if (tfod != null) {
            tfod.shutdown();
        }

        if (lineUp != null) {
            lineUp.Dispose();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() throws Exception {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        try {
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraDirection = CameraDirection.BACK;

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
        }
        catch (Exception ex){
            telemetry.addData("Issues with Vuforia initialization", ex);
            throw new Exception("Issues with Vuforia initialization", ex);
        }
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
