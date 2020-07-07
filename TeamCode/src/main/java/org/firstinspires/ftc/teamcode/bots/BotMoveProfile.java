package org.firstinspires.ftc.teamcode.bots;

import org.firstinspires.ftc.teamcode.calibration.MotorReductionBot;

public class BotMoveProfile {
    private double realSpeedLeft = 0;
    private double realSpeedRight = 0;
    private double longTarget = 0;
    private double slowdownMarkLong = 0;
    private double slowdownMarkShort = 0;
    private boolean leftLong;
    private double speedRatio = 1;
    private RobotDirection direction;
    private MotorReductionBot motorReduction = null;
    private BotMoveRequest target = new BotMoveRequest();


    @Override
    public String toString() {
        return String.format("Direction: %s \nL:%.2f  R:%.2f\n Slowdownns: %.2f %.2f", direction.name(), realSpeedLeft, realSpeedRight, slowdownMarkLong,slowdownMarkShort);
    }

    public double getRealSpeedLeft() {
        return realSpeedLeft;
    }

    public void setRealSpeedLeft(double realSpeedLeft) {
        this.realSpeedLeft = realSpeedLeft;
    }

    public double getRealSpeedRight() {
        return realSpeedRight;
    }

    public void setRealSpeedRight(double realSpeedRight) {
        this.realSpeedRight = realSpeedRight;
    }

    public double getLongTarget() {
        return longTarget;
    }

    public void setLongTarget(double longTarget) {
        this.longTarget = longTarget;
    }

    public double getSlowdownMarkLong() {
        return slowdownMarkLong;
    }

    public void setSlowdownMarkLong(double slowdownMarkLong) {
        this.slowdownMarkLong = slowdownMarkLong;
    }

    public double getSlowdownMarkShort() {
        return slowdownMarkShort;
    }

    public void setSlowdownMarkShort(double slowdownMarkShort) {
        this.slowdownMarkShort = slowdownMarkShort;
    }

    public boolean isLeftLong() {
        return leftLong;
    }

    public void setLeftLong(boolean leftLong) {
        this.leftLong = leftLong;
    }

    public MotorReductionBot getMotorReduction() {
        return motorReduction;
    }

    public void setMotorReduction(MotorReductionBot motorReduction) {
        this.motorReduction = motorReduction;
    }

    public RobotDirection getDirection() {
        return direction;
    }

    public void setDirection(RobotDirection direction) {
        this.direction = direction;
    }

    public double getSpeedRatio() {
        return speedRatio;
    }

    public void setSpeedRatio(double speedRatio) {
        this.speedRatio = speedRatio;
    }

    public BotMoveRequest getTarget() {
        return target;
    }

    public void setTarget(BotMoveRequest target) {
        this.target = target;
    }
}
