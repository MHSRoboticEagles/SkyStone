package org.firstinspires.ftc.teamcode.skills;

public class Geometry {

    public static double getDistance(double X, double Y, double targetX, double targetY){
        double currentX = X;
        double currentY = Y;

        //determine the new heading to the target
        double distanceX = Math.abs(targetX - currentX);
        double distanceY = Math.abs(targetY - currentY);
        double chord = Math.sqrt(distanceX*distanceX + distanceY * distanceY);
        return chord;
    }
}
