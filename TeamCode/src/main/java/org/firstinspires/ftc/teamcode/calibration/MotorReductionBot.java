package org.firstinspires.ftc.teamcode.calibration;

import java.io.Serializable;

import static org.firstinspires.ftc.teamcode.calibration.MotorReduction.DEFAULT_REDUCTION;

public class MotorReductionBot implements Serializable {

    protected int selectedIndex = 0;
    private double distanceRatio = 1;
    private double headChange = 0;

    protected MotorName [] motors = new MotorName[]{MotorName.LF, MotorName.RF, MotorName.RB, MotorName.LB};
    protected double [] MRs = new double[] {DEFAULT_REDUCTION, DEFAULT_REDUCTION, DEFAULT_REDUCTION, DEFAULT_REDUCTION};

    public MotorReductionBot(){

    }

    public double getLF() {
        return MRs[0];
    }

    public void setLF(double LF) {
        MRs[0] = LF;
    }

    public double getRF() {
        return MRs[1];
    }

    public void setRF(double RF) {
        MRs[1] = RF;
    }

    public double getRB() {
        return MRs[2];
    }

    public void setRB(double RB) {
        MRs[2] = RB;
    }

    public double getLB() {
        return MRs[3];
    }

    public void setLB(double LB) {
        MRs[3] = LB;
    }


    public boolean isSelected(int index){
        return this.selectedIndex == index;
    }


    public void inrementSelectedMR(){
        this.MRs[selectedIndex] = this.MRs[selectedIndex] + 0.01;
        if (this.MRs[selectedIndex] > 1){
            this.MRs[selectedIndex] = 1;
        }
    }

    public void decrementSelectedMR(){
        this.MRs[selectedIndex] = this.MRs[selectedIndex] - 0.01;
        if (this.MRs[selectedIndex] < 0){
            this.MRs[selectedIndex] = 0;
        }
    }

    public void selectNext(){
        if (selectedIndex + 1 >= motors.length){
            selectedIndex = 0;
        }
        else{
            selectedIndex++;
        }
    }

    public void selectPrev(){
        if (selectedIndex - 1 < 0){
            selectedIndex = motors.length - 1;
        }
        else{
            selectedIndex--;
        }
    }

    public double getDistanceRatio() {
        return distanceRatio;
    }

    public void setDistanceRatio(double distanceRatio) {
        this.distanceRatio = distanceRatio;
    }

    public double getHeadChange() {
        return headChange;
    }

    public void setHeadChange(double headChange) {
        this.headChange = headChange;
    }

    public boolean compare(MotorReductionBot another){
        if (another == null){
            return true;
        }
        return Math.abs(this.headChange) <= Math.abs(another.getHeadChange());
    }
}
