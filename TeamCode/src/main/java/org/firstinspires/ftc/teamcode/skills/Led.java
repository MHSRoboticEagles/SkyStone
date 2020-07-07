package org.firstinspires.ftc.teamcode.skills;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Led {
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    public void init(HardwareMap hwMap, Telemetry telemetry){
        blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "led");
        if (blinkinLedDriver == null){
            telemetry.addData("Led", "Failed to initialize");
        }
        else{
            telemetry.addData("Led", "Initialized OK");
        }
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

    public void breaking(){
        setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
    }

    public void problem(){
        setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
    }
}
