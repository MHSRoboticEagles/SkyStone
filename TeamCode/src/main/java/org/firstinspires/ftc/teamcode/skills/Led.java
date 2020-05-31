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
            pattern = p;
            blinkinLedDriver.setPattern(p);
        }
    }


    public void none(){
        setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    public void OK(){
        setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
    }

    public void needAdjustment(){
        setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
    }

    public void start(){
        this.OK();
    }

    public void move(){
        setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);
    }

    public void problem(){
        setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
    }
}
