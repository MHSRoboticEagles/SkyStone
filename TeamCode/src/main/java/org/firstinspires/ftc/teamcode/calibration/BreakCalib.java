package org.firstinspires.ftc.teamcode.calibration;

public class BreakCalib {
    private double breakPoint = 0;
    private double overRun = 0;
    private boolean isComplete = false;

    public double getBreakPoint() {
        return breakPoint;
    }

    public void setBreakPoint(double breakPoint) {
        this.breakPoint = breakPoint;
    }

    public double getOverRun() {
        return overRun;
    }

    public void setOverRun(double overRun) {
        this.overRun = overRun;
    }

    public boolean isComplete() {
        return isComplete;
    }

    public void setComplete(boolean complete) {
        isComplete = complete;
    }
}
