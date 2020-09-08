package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.bots.BotMoveProfile;
import org.firstinspires.ftc.teamcode.bots.MoveStrategy;

public class AutoStep implements Cloneable {
    private int waitMS = 0;
    private int targetX;
    private int targetY;
    private double topSpeed = 0.5;
    private MoveStrategy moveStrategy = MoveStrategy.Curve;
    private String action;
    private double desiredHead = BotMoveProfile.DEFAULT_HEADING;

    public int getWaitMS() {
        return waitMS;
    }

    public void setWaitMS(int waitMS) {
        this.waitMS = waitMS;
    }

    public int getTargetX() {
        return targetX;
    }

    public void setTargetX(int targetX) {
        this.targetX = targetX;
    }

    public int getTargetY() {
        return targetY;
    }

    public void setTargetY(int targetY) {
        this.targetY = targetY;
    }

    public double getTopSpeed() {
        return topSpeed;
    }

    public void setTopSpeed(double topSpeed) {
        this.topSpeed = topSpeed;
    }

    public MoveStrategy getMoveStrategy() {
        return moveStrategy;
    }

    public void setMoveStrategy(MoveStrategy moveStrategy) {
        this.moveStrategy = moveStrategy;
    }

    public String getAction() {
        return action;
    }

    public void setAction(String action) {
        this.action = action;
    }

    public String getDestination(){
        return String.format("%d : %d", getTargetX(), getTargetY());
    }

    public String getTopSpeedString(){
        return String.format("%.2f", getTopSpeed());
    }

    public String getMoveStrategyString(){
        return getMoveStrategy().name();
    }

    public String getWaitString(){
        return String.format("%d", getWaitMS());
    }

    public double getDesiredHead() {
        return desiredHead;
    }

    public String getDesiredHeadString() {
        return String.format("%.2f", desiredHead);
    }

    public void setDesiredHead(double desiredHead) {
        this.desiredHead = desiredHead;
    }

    public AutoStep clone(){
        AutoStep clone = new AutoStep();
        clone.setWaitMS(this.getWaitMS());
        clone.setTargetX(this.getTargetX());
        clone.setTargetY(this.getTargetY());
        clone.setTopSpeed(this.getTopSpeed());
        clone.setMoveStrategy(this.getMoveStrategy());
        clone.setAction(this.getAction());
        clone.setDesiredHead(this.getDesiredHead());
        return clone;
    }
}
