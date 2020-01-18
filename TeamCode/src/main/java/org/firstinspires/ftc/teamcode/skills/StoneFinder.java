package org.firstinspires.ftc.teamcode.skills;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


public class StoneFinder {
    private TFObjectDetector tfod;
    private float stoneLeft = -1;
    private float stoneWidth = -1;
    private float stoneTop = -1;
    private double angle = 0;
    private static final String LABEL_SKYSTONE = "Skystone";

    public StoneFinder(TFObjectDetector tf) {
        tfod = tf;
    }

    public boolean detect(Telemetry telemetry)

    {
        boolean found = false;
        telemetry.addData("Info", "Trying Detection");


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
                    found = true;
                    stoneLeft = recognition.getLeft();
                    stoneWidth = recognition.getWidth();
                    telemetry.addData("Found", found);
                    telemetry.addData("Left", getStoneLeft());
                    break;
                }
            }
        }
        telemetry.update();
        return found;
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
                            angle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
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


    protected void stopStoneDetection(){
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public float getStoneLeft() {
        return stoneLeft;
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
}
