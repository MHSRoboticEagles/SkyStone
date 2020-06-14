package org.firstinspires.ftc.teamcode.calibration;

import java.io.Serializable;

public class MotorReductionBot implements Serializable {
    private double LF = 1;
    private double LB = 1;
    private double RF = 1;
    private double RB = 1;

    public MotorReductionBot(){

    }

    public MotorReductionBot(MotorReductionCalib calib){
        if (calib.getMotorName() == MotorName.LF){
            setLF(calib.getMotorReduction());
        }
        else if (calib.getMotorName() == MotorName.LB){
            setLB(calib.getMotorReduction());
        }
        else if (calib.getMotorName() == MotorName.RF){
            setRF(calib.getMotorReduction());
        }
        else if (calib.getMotorName() == MotorName.RB){
            setRB(calib.getMotorReduction());
        }
    }

    public double getLF() {
        return LF;
    }

    public void setLF(double LF) {
        this.LF = LF;
    }

    public double getLB() {
        return LB;
    }

    public void setLB(double LB) {
        this.LB = LB;
    }

    public double getRF() {
        return RF;
    }

    public void setRF(double RF) {
        this.RF = RF;
    }

    public double getRB() {
        return RB;
    }

    public void setRB(double RB) {
        this.RB = RB;
    }
}
