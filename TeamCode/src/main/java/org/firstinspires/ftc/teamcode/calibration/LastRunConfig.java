package org.firstinspires.ftc.teamcode.calibration;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;

import java.io.Serializable;

public class LastRunConfig implements Serializable {
    private double X;
    private double Y;
    private double heading;
    private double speed;


    public String serialize() {
        return SimpleGson.getInstance().toJson(this);
    }
    public static LastRunConfig deserialize(String data) {
        return SimpleGson.getInstance().fromJson(data, LastRunConfig.class);
    }

    public double getX() {
        return X;
    }

    public void setX(double x) {
        X = x;
    }

    public double getY() {
        return Y;
    }

    public void setY(double y) {
        Y = y;
    }

    public double getHeading() {
        return heading;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    public double getSpeed() {
        return speed;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }
}
