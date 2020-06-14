package org.firstinspires.ftc.teamcode.calibration;

import org.firstinspires.ftc.teamcode.bots.RobotVeer;

import java.util.HashMap;
import java.util.Map;

public class MotorReductionCalib extends MotorReduction {

    public static double DEFAULT_REDUCTION_STEP = 0.1;
    private double headChange = 180;
    private double originalHeadChange = 180;
    private double reductionStep = DEFAULT_REDUCTION_STEP;
    private boolean calibComplete = false;
    private HashMap<Double, Double> redMap = new HashMap<>();

    private RobotVeer veer = RobotVeer.NONE;


    private double leftOdoDistance;
    private double leftOdoDistanceActual;

    private double rightOdoDistance;
    private double rightOdoDistanceActual;

    private double horOdoDistance;
    private double horOdoDistanceActual;

    private double breakPointLeft;
    private double breakPointRight;
    private double breakPointHor;

    public MotorReductionCalib(){

    }

    public void setMotorName(MotorName motorName) {
            this.motorName = motorName;
    }

    public MotorReductionCalib(MotorName name) {
        super(name);
    }

    public double getHeadChange() {
        return headChange;
    }

    public void setHeadChange(double headChange) {
        this.headChange = headChange;
    }

    public double getReductionStep() {
        return reductionStep;
    }

    public void setReductionStep(double reductionStep) {
        this.reductionStep = reductionStep;
    }

    public void adjustReduction(){
        setMotorReduction(getMotorReduction() - getReductionStep());
    }


    public  MotorReduction getMR(){
        return new MotorReduction(this.getMotorName(), this.getMotorReduction());
    }

    public double getOriginalHeadChange() {
        return originalHeadChange;
    }

    public void setOriginalHeadChange(double originalHeadChange) {
        this.originalHeadChange = originalHeadChange;
    }

    public boolean isCalibComplete() {
        return calibComplete;
    }

    public void setCalibComplete(boolean calibComplete) {
        this.calibComplete = calibComplete;
    }

    public void setReductionForSpeed(double reduction, double speed){
        this.redMap.put(speed, reduction);
    }

    public Double getReductionForSpeed(double speed){
        if (this.redMap.containsKey(speed)){
            return this.redMap.get(speed);
        }
        return null;
    }

    public void computeSpeedReduction(){
//        double min = 1;
//        double max = 0;
//        double minVal = 0;
//        double maxVal = 0;
//        for (Map.Entry<Double, Double> entry : redMap.entrySet()) {
//            if (entry.getKey() >= max ){
//                max = entry.getKey();
//                maxVal = entry.getValue();
//            }
//            if (entry.getKey() <= min){
//                min = entry.getKey();
//                minVal = entry.getValue();
//            }
//        }
//
//        double coeff = (max - min)/DEFAULT_REDUCTION_STEP;
//        double perStepReduction = Math.abs(maxVal - minVal)/coeff;
//        this.setMotorReduction(perStepReduction);

        double coeff = getCalibSpeed()/DEFAULT_REDUCTION_STEP;
        this.setMotorReductionPerSpeedIncrement(this.getMotorReduction()/coeff);
    }

    public void computeBreakDistance(){
        //break distance
        double overLeft = this.getLeftOdoDistanceActual() - this.getLeftOdoDistance();
        double overRight = this.getRightOdoDistanceActual() - this.getRightOdoDistance();
        double overHor = this.getHorOdoDistanceActual() - this.getHorOdoDistance();
        if (overLeft > 0){
            this.setBreakPointLeft(100 - overLeft/this.getLeftOdoDistance());
        }

        if (overRight > 0){
            this.setBreakPointRight(100 - overRight/this.getRightOdoDistance());
        }

        if (overHor > 0){
            this.setBreakPointHor(100 - overHor/this.getLeftOdoDistance());
        }
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(this.getMotorName());
        sb.append("  ");
        sb.append(String.format("Head: %.2f", this.getHeadChange()));
        sb.append("  ");
        sb.append(String.format("Break : %.2f; : %.2f; : %.2f", this.getBreakPointLeft(), this.getBreakPointRight(), this.getBreakPointHor()));
        sb.append(String.format("MR: %.2f; MR step: %.4f", this.getMotorReduction(), this.getMotorReductionPerSpeedIncrement()));

//        for (Map.Entry<Double, Double> entry : redMap.entrySet()) {
//            sb.append(String.format("%.2f: %.2f ; ", entry.getKey(), entry.getValue()));
//        }

        return sb.toString();
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

    public double getBreakPointLeft() {
        return breakPointLeft;
    }

    public void setBreakPointLeft(double breakPointLeft) {
        this.breakPointLeft = breakPointLeft;
    }

    public double getBreakPointRight() {
        return breakPointRight;
    }

    public void setBreakPointRight(double breakPointRight) {
        this.breakPointRight = breakPointRight;
    }

    public double getBreakPointHor() {
        return breakPointHor;
    }

    public void setBreakPointHor(double breakPointHor) {
        this.breakPointHor = breakPointHor;
    }

    public RobotVeer getVeer() {
        return veer;
    }

    public void setVeer(RobotVeer veer) {
        this.veer = veer;
    }

    public void computeReduction(){
        if (leftOdoDistanceActual > rightOdoDistanceActual){
            double left = rightOdoDistanceActual/leftOdoDistanceActual;
            this.setMotorReduction(left);
            this.setVeer(RobotVeer.RIGHT);
        }
        if (rightOdoDistanceActual > leftOdoDistanceActual){
            double right = leftOdoDistanceActual/rightOdoDistanceActual;
            this.setMotorReduction(right);
            this.setVeer(RobotVeer.LEFT);
        }
    }
}
