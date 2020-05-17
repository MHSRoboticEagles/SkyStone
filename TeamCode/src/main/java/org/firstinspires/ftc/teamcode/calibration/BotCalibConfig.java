package org.firstinspires.ftc.teamcode.calibration;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;

import java.io.Serializable;

public class BotCalibConfig implements Serializable {
    public static String BOT_CALIB_CONFIG = "bot-config.json";
    private double leftTickPerDegree;
    private double rightTickPerDegree;
    private double wheelBaseSeparation;
    private double horizontalTicksDegree;
    private double minRadiusLeft;
    private double minRadiusRight;
    private double strafeLeftReduction;
    private double strafeRightReduction;

    public String serialize() {
        return SimpleGson.getInstance().toJson(this);
    }
    public static BotCalibConfig deserialize(String data) {
        return SimpleGson.getInstance().fromJson(data, BotCalibConfig.class);
    }

    public double getLeftTickPerDegree() {
        return leftTickPerDegree;
    }

    public void setLeftTickPerDegree(double leftTickPerDegree) {
        this.leftTickPerDegree = leftTickPerDegree;
    }

    public double getRightTickPerDegree() {
        return rightTickPerDegree;
    }

    public void setRightTickPerDegree(double rightTickPerDegree) {
        this.rightTickPerDegree = rightTickPerDegree;
    }

    public double getWheelBaseSeparation() {
        return wheelBaseSeparation;
    }

    public void setWheelBaseSeparation(double wheelBaseSeparation) {
        this.wheelBaseSeparation = wheelBaseSeparation;
    }

    public double getHorizontalTicksDegree() {
        return horizontalTicksDegree;
    }

    public void setHorizontalTicksDegree(double horizontalTicksDegree) {
        this.horizontalTicksDegree = horizontalTicksDegree;
    }

    public double getMinRadiusLeft() {
        return minRadiusLeft;
    }

    public void setMinRadiusLeft(double minRadiusLeft) {
        this.minRadiusLeft = minRadiusLeft;
    }

    public double getMinRadiusRight() {
        return minRadiusRight;
    }

    public void setMinRadiusRight(double minRadiusRight) {
        this.minRadiusRight = minRadiusRight;
    }

    public double getStrafeLeftReduction() {
        return strafeLeftReduction;
    }

    public void setStrafeLeftReduction(double strafeLeftReduction) {
        this.strafeLeftReduction = strafeLeftReduction;
    }

    public double getStrafeRightReduction() {
        return strafeRightReduction;
    }

    public void setStrafeRightReduction(double strafeRightReduction) {
        this.strafeRightReduction = strafeRightReduction;
    }
}
