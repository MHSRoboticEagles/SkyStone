package org.firstinspires.ftc.teamcode.calibration;

public class MotorReduction {

    public static double DEFAULT_REDUCTION = 1;
    protected MotorName motorName = MotorName.NONE;
    private double motorReduction = DEFAULT_REDUCTION;
    private double calibSpeed = 0;

    private double motorReductionPerSpeedIncrement = DEFAULT_REDUCTION;

    public MotorReduction(){

    }

    public MotorReduction(MotorName name) {
        this(name, 1);

    }

    public MotorReduction(MotorName name, double reduction){
        this.setMotorReduction(reduction);
    }

    public double getMotorReduction() {
        return motorReduction;
    }

    public void setMotorReduction(double motorReduction) {
        this.motorReduction = motorReduction;
    }

    public MotorName getMotorName() {
        return motorName;
    }

    public void setMotorName(MotorName motorName) {
        //set only once
        if (this.getMotorName() == MotorName.NONE) {
            this.motorName = motorName;
        }
    }

    public double getCalibSpeed() {
        return calibSpeed;
    }

    public void setCalibSpeed(double calibSpeed) {
        this.calibSpeed = calibSpeed;
    }

    public double getMotorReductionPerSpeedIncrement() {
        return motorReductionPerSpeedIncrement;
    }

    public void setMotorReductionPerSpeedIncrement(double motorReductionPerSpeedIncrement) {
        this.motorReductionPerSpeedIncrement = motorReductionPerSpeedIncrement;
    }
}

