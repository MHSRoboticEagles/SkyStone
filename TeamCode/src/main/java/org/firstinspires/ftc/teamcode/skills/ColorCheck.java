package org.firstinspires.ftc.teamcode.skills;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by sjeltuhin on 9/26/17.
 */

public class ColorCheck {
    NormalizedColorSensor colorSensor;
    ElapsedTime runtime = new ElapsedTime();

    public void init(HardwareMap hardwareMap){
        // Get a reference to our sensor object.
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        toggleLight(true);
    }

    private void toggleLight(boolean on){
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(on);
        }
    }


    public DetectedColor detectColor(Telemetry telemetry, float timeout, LinearOpMode caller) {

        DetectedColor dc = DetectedColor.NONE;
        int SCALE = 255;

        runtime.reset();
        boolean stop = false;
        while(!stop) {
                    // Read the sensor
            NormalizedRGBA colors =  colorSensor.getNormalizedColors();

            telemetry.addLine()
                    .addData("r", "%.2f", colors.red * SCALE )
                    .addData("g", "%.2f", colors.green * SCALE)
                    .addData("b", "%.2f", colors.blue * SCALE);


            telemetry.update();

            stop = !caller.opModeIsActive();
        }
        return dc;
    }
}
