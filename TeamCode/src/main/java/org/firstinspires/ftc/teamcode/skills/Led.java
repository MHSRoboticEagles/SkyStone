package org.firstinspires.ftc.teamcode.skills;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Led {
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    public void init(HardwareMap hwMap){
        blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "led");

    }

    protected void setPattern(RevBlinkinLedDriver.BlinkinPattern p){
        if (blinkinLedDriver != null){
            blinkinLedDriver.setPattern(p);
        }
    }

    public void start(){
        setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
    }

    public void problem(){
        setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
    }
}
