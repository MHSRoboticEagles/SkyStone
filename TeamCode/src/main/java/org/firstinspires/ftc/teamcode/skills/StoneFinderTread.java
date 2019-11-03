package org.firstinspires.ftc.teamcode.skills;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


public class StoneFinderTread implements Runnable {
    DetectionInterface callback = null;
    private boolean done = false;


    private TFObjectDetector tfod;

    public StoneFinderTread(DetectionInterface c, TFObjectDetector tf) {
        callback = c;
        tfod = tf;
    }

    @Override
    public void run() {
        float left = -1;

        left = detectStone();

//        if (this.callback != null) {
//            this.callback.stoneFound(left);
//        }
    }

    public void stop(){
        this.done = true;
        stopStoneDetection();
    }



    protected float detectStone(){
        float left = -1;

        while (!this.done) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {

                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        left = recognition.getLeft();
                        break;
                    }
                }
            }
            if (left > -1){
                this.done = true;
            }

        }
        return left;
    }

    protected float detect()

    {
        float left = -1;


        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
//                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
//                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
//                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
//                                recognition.getLeft(), recognition.getTop());
//                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
//                                recognition.getRight(), recognition.getBottom());
                    left = recognition.getLeft();
                    break;
                }
//                    telemetry.update();
            }
        }
        if (left > -1) {
            this.done = true;
        }
        return left;
    }


    protected void stopStoneDetection(){
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}
