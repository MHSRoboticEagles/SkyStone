package org.firstinspires.ftc.teamcode.calibration;

import org.firstinspires.ftc.teamcode.bots.RobotDirection;
import org.firstinspires.ftc.teamcode.bots.RobotVeer;

import java.io.Serializable;

public class MotorReductionBotCalib implements Serializable {
    private double LF = 1;
    private double LB = 1;
    private double RF = 1;
    private double RB = 1;

    private double headChangeBaseline = 0;
    private double MRBaseline = 1;
    private RobotVeer veer = RobotVeer.NONE;
    private RobotDirection direction = RobotDirection.Forward;

    private double LFHeadChange = 0;
    private double LBHeadChange = 0;
    private double RFHeadChange = 0;
    private double RBHeadChange = 0;

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

    public double getLFHeadChange() {
        return LFHeadChange;
    }

    public void setLFHeadChange(double LFHeadChange) {
        this.LFHeadChange = LFHeadChange;
    }

    public double getLBHeadChange() {
        return LBHeadChange;
    }

    public void setLBHeadChange(double LBHeadChange) {
        this.LBHeadChange = LBHeadChange;
    }

    public double getRFHeadChange() {
        return RFHeadChange;
    }

    public void setRFHeadChange(double RFHeadChange) {
        this.RFHeadChange = RFHeadChange;
    }

    public double getRBHeadChange() {
        return RBHeadChange;
    }

    public void setRBHeadChange(double RBHeadChange) {
        this.RBHeadChange = RBHeadChange;
    }

    public void update(MotorReductionCalib calib){
        if (calib.getMotorName() == MotorName.LF){
            setLF(calib.getMotorReduction());
            setLFHeadChange(calib.getHeadChange());
        }
        else if (calib.getMotorName() == MotorName.LB){
            setLB(calib.getMotorReduction());
            setLBHeadChange(calib.getHeadChange());
        }
        else if (calib.getMotorName() == MotorName.RF){
            setRF(calib.getMotorReduction());
            setRFHeadChange(calib.getHeadChange());
        }
        else if (calib.getMotorName() == MotorName.RB){
            setRB(calib.getMotorReduction());
            setRBHeadChange(calib.getHeadChange());
        }
    }

    public double getHeadChangeBaseline() {
        return headChangeBaseline;
    }

    public void setHeadChangeBaseline(double headChangeOriginal) {
        this.headChangeBaseline = headChangeOriginal;
    }

    public RobotVeer getVeer() {
        return veer;
    }

    public void setVeer(RobotVeer veer) {
        this.veer = veer;
    }

    public RobotDirection getDirection() {
        return direction;
    }

    public void setDirection(RobotDirection direction) {
        this.direction = direction;
    }

    public double getMRBaseline() {
        return MRBaseline;
    }

    public void setMRBaseline(double MRBaseline) {
        this.MRBaseline = MRBaseline;
    }

    public MotorReductionCalib analyze(){
        double bestChange = Math.abs(getHeadChangeBaseline());
        double effectiveMR = getMRBaseline();
        double remainingMR = effectiveMR;
        MotorReductionCalib calib = new MotorReductionCalib();
        if (Math.abs(this.LFHeadChange) > 0 && Math.abs(this.LFHeadChange) < bestChange){
            bestChange = Math.abs(LFHeadChange);
            calib.setMotorName(MotorName.LF);
        }
        if (Math.abs(this.LBHeadChange) > 0 && Math.abs(this.LBHeadChange) < bestChange){
            bestChange = Math.abs(LBHeadChange);
            calib.setMotorName(MotorName.LB);
        }

        if (Math.abs(this.RFHeadChange) > 0 && Math.abs(this.RFHeadChange) < bestChange){
            bestChange = Math.abs(RFHeadChange);
            calib.setMotorName(MotorName.RF);
        }

        if (Math.abs(this.RBHeadChange) > 0 && Math.abs(this.RBHeadChange) < bestChange){
            bestChange = Math.abs(RBHeadChange);
            calib.setMotorName(MotorName.RB);
        }

        double changeEffect = (Math.abs(headChangeBaseline) - bestChange)/Math.abs(headChangeBaseline);
        double adjustedMR = 1 - ((1 - effectiveMR)/changeEffect);
        calib.setMotorReduction(adjustedMR);

        return calib;

    }
}
