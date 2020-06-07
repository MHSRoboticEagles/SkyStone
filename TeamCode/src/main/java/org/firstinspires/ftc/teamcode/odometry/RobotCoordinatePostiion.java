package org.firstinspires.ftc.teamcode.odometry;

import org.firstinspires.ftc.teamcode.bots.YellowBot;
import org.firstinspires.ftc.teamcode.calibration.BotCalibConfig;

public class RobotCoordinatePostiion implements Runnable {

    BotCalibConfig config;
    YellowBot bot;
    private boolean isRunning = true;
    private int sleepTime;

    private double robotEncoderWheelDistance;
    private double horizontalEncoderTickPerDegreeOffset;

    double verticalRightEncoderWheelPosition = 0, verticalLeftEncoderWheelPosition = 0, horEncoderWheelPosition = 0,  changeInRobotOrientation = 0;
    private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;
    private double previousVerticalRightEncoderWheelPosition = 0, previousVerticalLeftEncoderWheelPosition = 0, prevNormalEncoderWheelPosition = 0;

    private int verticalLeftEncoderPositionMultiplier = 1;
    private int verticalRightEncoderPositionMultiplier = 1;
    private int horEncoderPositionMultiplier = 1;

    public RobotCoordinatePostiion(YellowBot bot, int sleepTimeMS){
        this.bot = bot;
        config = bot.getCalibConfig();
        sleepTime = sleepTimeMS;
        this.robotEncoderWheelDistance = config.getWheelBaseSeparation() * bot.COUNTS_PER_INCH_REV;
        this.horizontalEncoderTickPerDegreeOffset = config.getHorizontalTicksDegree();
    }


    private void updatePostition(){
        verticalLeftEncoderWheelPosition = (bot.getLeftOdemeter() * verticalLeftEncoderPositionMultiplier);
        verticalRightEncoderWheelPosition = (bot.getRightOdemeter() * verticalRightEncoderPositionMultiplier);

        double leftChange = verticalLeftEncoderWheelPosition - previousVerticalLeftEncoderWheelPosition;
        double rightChange = verticalRightEncoderWheelPosition - previousVerticalRightEncoderWheelPosition;

        changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
        robotOrientationRadians = ((robotOrientationRadians + changeInRobotOrientation));

        horEncoderWheelPosition = (bot.getHorizontalOdemeter() * horEncoderPositionMultiplier) - (changeInRobotOrientation*horizontalEncoderTickPerDegreeOffset);
        double horizontalChange = horEncoderWheelPosition - prevNormalEncoderWheelPosition;


        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange;

        robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (p*Math.sin(robotOrientationRadians) + n*Math.cos(robotOrientationRadians));
        robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (p*Math.cos(robotOrientationRadians) - n*Math.sin(robotOrientationRadians));

        previousVerticalLeftEncoderWheelPosition = verticalLeftEncoderWheelPosition;
        previousVerticalRightEncoderWheelPosition = verticalRightEncoderWheelPosition;
        prevNormalEncoderWheelPosition = horEncoderWheelPosition;


    }

    @Override
    public void run() {
        while(isRunning) {
            updatePostition();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }


    public void stop(){ isRunning = false; }

    public double getX(){ return robotGlobalXCoordinatePosition; }


    public double getY(){ return robotGlobalYCoordinatePosition; }

    public double getXInches(){ return robotGlobalXCoordinatePosition/bot.COUNTS_PER_INCH_REV; }


    public double getYInches(){ return robotGlobalYCoordinatePosition/bot.COUNTS_PER_INCH_REV; }


    public double returnOrientation(){ return Math.toDegrees(robotOrientationRadians) % 360; }


    public void reverseLeftEncoder(){
        if(verticalLeftEncoderPositionMultiplier == 1){
            verticalLeftEncoderPositionMultiplier = -1;
        }else{
            verticalLeftEncoderPositionMultiplier = 1;
        }
    }

    public void reverseRightEncoder(){
        if(verticalRightEncoderPositionMultiplier == 1){
            verticalRightEncoderPositionMultiplier = -1;
        }else{
            verticalRightEncoderPositionMultiplier = 1;
        }
    }

    public void reverseHorEncoder(){
        if(horEncoderPositionMultiplier == 1){
            horEncoderPositionMultiplier = -1;
        }else{
            horEncoderPositionMultiplier = 1;
        }
    }
}
