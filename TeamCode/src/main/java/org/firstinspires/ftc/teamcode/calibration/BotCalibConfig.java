package org.firstinspires.ftc.teamcode.calibration;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;

import java.io.Serializable;

public class BotCalibConfig implements Serializable {
    public static String BOT_CALIB_CONFIG = "bot-config.json";
    private double leftTickPerDegree;
    private double rightTickPerDegree;
    private double wheelBaseSeparation;

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
}
