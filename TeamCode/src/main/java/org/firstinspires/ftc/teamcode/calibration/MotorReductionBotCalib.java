package org.firstinspires.ftc.teamcode.calibration;

import org.firstinspires.ftc.teamcode.bots.RobotDirection;
import org.firstinspires.ftc.teamcode.bots.RobotVeer;

import java.io.Serializable;


public class MotorReductionBotCalib extends MotorReductionBot implements Serializable {

    private double headChangeBaseline = 0;
    private double MRBaseline = 1;
    private RobotVeer veer = RobotVeer.NONE;
    private RobotDirection direction = RobotDirection.Forward;

    private double LFHeadChange = 0;
    private double LBHeadChange = 0;
    private double RFHeadChange = 0;
    private double RBHeadChange = 0;

    private double originalHeadChange = 0;

    private double leftOdoDistance;
    private double leftOdoDistanceActual;

    private double rightOdoDistance;
    private double rightOdoDistanceActual;

    private double horOdoDistance;
    private double horOdoDistanceActual;

    public MotorReductionBotCalib(){

    }

    public MotorReductionBotCalib(MotorReductionBot mrb){
        if (mrb != null) {
            for (int x = 0; x < mrb.motors.length; x++) {
                this.MRs[x] = mrb.MRs[x];
            }
        }
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

//    public void update(MotorReductionCalib calib){
//        if (calib.getMotorName() == MotorName.LF){
//            setLF(calib.getMotorReduction());
//            setLFHeadChange(calib.getHeadChange());
//        }
//        else if (calib.getMotorName() == MotorName.LB){
//            setLB(calib.getMotorReduction());
//            setLBHeadChange(calib.getHeadChange());
//        }
//        else if (calib.getMotorName() == MotorName.RF){
//            setRF(calib.getMotorReduction());
//            setRFHeadChange(calib.getHeadChange());
//        }
//        else if (calib.getMotorName() == MotorName.RB){
//            setRB(calib.getMotorReduction());
//            setRBHeadChange(calib.getHeadChange());
//        }
//        else if (calib.getMotorName() == MotorName.LEFT_SIDE){
//            setLF(calib.getMotorReduction());
//            setLFHeadChange(calib.getHeadChange());
//            setLB(calib.getMotorReduction());
//            setLBHeadChange(calib.getHeadChange());
//        }
//        else if (calib.getMotorName() == MotorName.RIGHT_SIDE){
//            setRF(calib.getMotorReduction());
//            setRFHeadChange(calib.getHeadChange());
//            setRB(calib.getMotorReduction());
//            setRBHeadChange(calib.getHeadChange());
//        }
//    }

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

//    public MotorReductionCalib analyze(){
//        double bestChange = Math.abs(getHeadChangeBaseline());
//        double effectiveMR = getMRBaseline();
//        double remainingMR = effectiveMR;
//        MotorReductionCalib calib = new MotorReductionCalib();
//        if (Math.abs(this.LFHeadChange) > 0 && Math.abs(this.LFHeadChange) < bestChange){
//            bestChange = Math.abs(LFHeadChange);
//            calib.setMotorName(MotorName.LF);
//        }
//        if (Math.abs(this.LBHeadChange) > 0 && Math.abs(this.LBHeadChange) < bestChange){
//            bestChange = Math.abs(LBHeadChange);
//            calib.setMotorName(MotorName.LB);
//        }
//
//        if (Math.abs(this.RFHeadChange) > 0 && Math.abs(this.RFHeadChange) < bestChange){
//            bestChange = Math.abs(RFHeadChange);
//            calib.setMotorName(MotorName.RF);
//        }
//
//        if (Math.abs(this.RBHeadChange) > 0 && Math.abs(this.RBHeadChange) < bestChange){
//            bestChange = Math.abs(RBHeadChange);
//            calib.setMotorName(MotorName.RB);
//        }
//
//        double changeEffect = (Math.abs(headChangeBaseline) - bestChange)/Math.abs(headChangeBaseline);
//        double adjustedMR = 1 - ((1 - effectiveMR)/changeEffect);
//        calib.setMotorReduction(adjustedMR);
//
//        return calib;
//    }

    public MotorReductionBot getMR(){
        MotorReductionBot mrb = new MotorReductionBot();
        for(int x = 0; x < motors.length; x++){
            mrb.MRs[x] = this.MRs[x];
        }
        mrb.setHeadChange(this.getHeadChange());
        mrb.setDistanceRatio(this.getDistanceRatio());
        return mrb;
    }


    public double getOriginalHeadChange() {
        return originalHeadChange;
    }

    public void setOriginalHeadChange(double originalHeadChange) {
        this.originalHeadChange = originalHeadChange;
    }

    public double getLeftOdoDistance() {
        return leftOdoDistance;
    }

    public void setLeftOdoDistance(double leftOdoDistance) {
        this.leftOdoDistance = leftOdoDistance;
    }

    public double getLeftOdoDistanceActual() {
        return leftOdoDistanceActual;
    }

    public void setLeftOdoDistanceActual(double leftOdoDistanceActual) {
        this.leftOdoDistanceActual = leftOdoDistanceActual;
    }

    public double getRightOdoDistance() {
        return rightOdoDistance;
    }

    public void setRightOdoDistance(double rightOdoDistance) {
        this.rightOdoDistance = rightOdoDistance;
    }

    public double getRightOdoDistanceActual() {
        return rightOdoDistanceActual;
    }

    public void setRightOdoDistanceActual(double rightOdoDistanceActual) {
        this.rightOdoDistanceActual = rightOdoDistanceActual;
    }

    public double getHorOdoDistance() {
        return horOdoDistance;
    }

    public void setHorOdoDistance(double horOdoDistance) {
        this.horOdoDistance = horOdoDistance;
    }

    public double getHorOdoDistanceActual() {
        return horOdoDistanceActual;
    }

    public void setHorOdoDistanceActual(double horOdoDistanceActual) {
        this.horOdoDistanceActual = horOdoDistanceActual;
    }

    public void process(boolean side){
        if (side){
            double linearMovement = Math.abs(leftOdoDistanceActual);
            this.setVeer(RobotVeer.RIGHT);
            if (linearMovement < Math.abs(rightOdoDistanceActual)){
                linearMovement = Math.abs(rightOdoDistanceActual);
                this.setVeer(RobotVeer.LEFT);
            }
            //make it negative for comparison: the larger the value, the better. 0 is the best
            setDistanceRatio(-linearMovement);
        }
        else {
            if (Math.abs(leftOdoDistanceActual) > Math.abs(rightOdoDistanceActual)) {
                double left = Math.abs(rightOdoDistanceActual / leftOdoDistanceActual);
                setDistanceRatio(left);
                this.setVeer(RobotVeer.RIGHT);
            }
            if (Math.abs(rightOdoDistanceActual) > Math.abs(leftOdoDistanceActual)) {
                double right = Math.abs(leftOdoDistanceActual / rightOdoDistanceActual);
                setDistanceRatio(right);
                this.setVeer(RobotVeer.LEFT);
            }
        }
    }


    public String getSelectedIndicator(int index){
        if (this.selectedIndex == index){
            return "*";
        }
        return " ";
    }
}
