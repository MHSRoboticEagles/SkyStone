package org.firstinspires.ftc.teamcode.odometry;

import android.graphics.Point;

import org.firstinspires.ftc.teamcode.bots.BotMoveProfile;
import org.firstinspires.ftc.teamcode.bots.BotMoveRequest;
import org.firstinspires.ftc.teamcode.bots.MoveStrategy;
import org.firstinspires.ftc.teamcode.bots.RobotDirection;
import org.firstinspires.ftc.teamcode.bots.YellowBot;
import org.firstinspires.ftc.teamcode.calibration.BotCalibConfig;

public class RobotCoordinatePosition implements Runnable {

    BotCalibConfig config;
    YellowBot bot;
    private boolean isRunning = true;
    private int sleepTime;
    private double realSpeedLeft = 0;
    private double realSpeedRight = 0;
    private double longTarget = 0;
    private double slowdownMarkLong = 0;
    private double slowdownMarkShort = 0;
    private boolean leftLong;

    private double robotEncoderWheelDistance;
    private double horizontalEncoderTickPerDegreeOffset;

    double verticalRightEncoderWheelPosition = 0, verticalLeftEncoderWheelPosition = 0, horEncoderWheelPosition = 0,  changeInRobotOrientation = 0;
    private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;
    private double frontCenterX = 0;
    private double frontCenterY = 0;
    private double previousVerticalRightEncoderWheelPosition = 0, previousVerticalLeftEncoderWheelPosition = 0, prevNormalEncoderWheelPosition = 0;

    private int verticalLeftEncoderPositionMultiplier = 1;
    private int verticalRightEncoderPositionMultiplier = 1;
    private int horEncoderPositionMultiplier = 1;
    private BotMoveRequest target = null;
    private double initialOrientation = 0;

    private double botHalfLength = bot.ROBOT_CENTER_Y* bot.COUNTS_PER_INCH_REV;

    public RobotCoordinatePosition(YellowBot bot, Point startPos, double initialOrientation, int sleepTimeMS){
        this.bot = bot;
        config = bot.getCalibConfig();
        sleepTime = sleepTimeMS;
        init(startPos, initialOrientation);
    }

    public void init(Point startPos, double initialOrientation){
        this.setInitialOrientation(initialOrientation);
        this.robotGlobalXCoordinatePosition = startPos.x * bot.COUNTS_PER_INCH_REV;
        this.robotGlobalYCoordinatePosition = startPos.y * bot.COUNTS_PER_INCH_REV;
        this.robotEncoderWheelDistance = config.getWheelBaseSeparation() * bot.COUNTS_PER_INCH_REV;
        this.horizontalEncoderTickPerDegreeOffset = config.getHorizontalTicksDegree();
        this.robotOrientationRadians = Math.toRadians(initialOrientation);
    }


    private void updatePosition(){
        verticalLeftEncoderWheelPosition = (bot.getLeftOdometer() * verticalLeftEncoderPositionMultiplier);
        verticalRightEncoderWheelPosition = (bot.getRightOdometer() * verticalRightEncoderPositionMultiplier);

        double leftChange = verticalLeftEncoderWheelPosition - previousVerticalLeftEncoderWheelPosition;
        double rightChange = verticalRightEncoderWheelPosition - previousVerticalRightEncoderWheelPosition;

        changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
        robotOrientationRadians = ((robotOrientationRadians + changeInRobotOrientation));

        horEncoderWheelPosition = (bot.getHorizontalOdometer() * horEncoderPositionMultiplier) - (changeInRobotOrientation*horizontalEncoderTickPerDegreeOffset);
        double horizontalChange = horEncoderWheelPosition - prevNormalEncoderWheelPosition;


        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange;

        robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (p*Math.sin(robotOrientationRadians) + n*Math.cos(robotOrientationRadians));
        robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (p*Math.cos(robotOrientationRadians) - n*Math.sin(robotOrientationRadians));

        //Front Center of the robot
        setFrontCenterX(robotGlobalXCoordinatePosition + botHalfLength*Math.sin(robotOrientationRadians));
        setFrontCenterY(robotGlobalYCoordinatePosition + botHalfLength*Math.cos(robotOrientationRadians));

        previousVerticalLeftEncoderWheelPosition = verticalLeftEncoderWheelPosition;
        previousVerticalRightEncoderWheelPosition = verticalRightEncoderWheelPosition;
        prevNormalEncoderWheelPosition = horEncoderWheelPosition;
    }

    @Override
    public void run() {
        while(isRunning) {
            updatePosition();
            if(target != null){
                adjustRoute();
            }
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }


    public void adjustRoute(){
        BotMoveProfile profile = BotMoveProfile.bestRoute(this.bot, getXInches(), getYInches(), this.target.getTarget(), this.target.getDirection(), this.target.getTopSpeed(), MoveStrategy.Curve,
        this);
        realSpeedLeft = profile.getRealSpeedLeft();
        realSpeedRight = profile.getRealSpeedRight();
        this.leftLong = profile.isLeftLong();
        this.slowdownMarkLong = profile.getSlowdownMarkLong();
        this.slowdownMarkShort = profile.getSlowdownMarkShort();
        this.longTarget = profile.getLongTarget();
    }

    public void adjustCurve(){
        try {
            double currentX = getXInches();
            double currentY = getYInches();
            double currentHead = this.getOrientation();
            boolean clockwise = currentHead >= 0;
            if (!clockwise) {
                currentHead = 360 + currentHead;
            }

            if (this.target.getDirection() == RobotDirection.Backward) {
                currentHead = (currentHead + 180) % 360;
            }

            boolean currentHeadInSquare4 = currentHead >= 270 && currentHead <= 360;
            boolean currentHeadInSquare1 = currentHead >= 0 && currentHead <= 90;

            //determine the new heading to the target
            double distanceX = this.target.getTarget().x - currentX;
            double distanceY = this.target.getTarget().y - currentY;
            double targetVector = Math.toDegrees(Math.atan2(distanceY, distanceX));


            if (distanceY == 0) {
                if (distanceX < 0) {
                    targetVector = 270;
                } else {
                    targetVector = 90;
                }
            } else if (distanceX == 0) {
                if (distanceY < 0) {
                    targetVector = 180;
                } else {
                    targetVector = 0;
                }
            } else {
                //lower left
                if (distanceX < 0 && distanceY < 0) {
                    targetVector = (-targetVector) + 90;
                }
                //lower right
                if (distanceX > 0 && distanceY < 0) {
                    targetVector = (-targetVector) + 90;
                }
                //upper right
                if (distanceX > 0 && distanceY > 0) {
                    targetVector = 90 - targetVector;
                }
                //upper left
                if (distanceX < 0 && distanceY > 0) {
                    targetVector = 360 - (targetVector - 90);
                }
            }

            boolean targetVectorInSquare1 = targetVector >= 0 && targetVector <= 90;
            boolean targetVectorInSquare4 = targetVector >= 270 && targetVector <= 360;

            double chord = Math.sqrt(distanceX * distanceX + distanceY * distanceY);


            double angleChange = Math.abs(targetVector - currentHead);
            if (targetVectorInSquare1 && currentHeadInSquare4 || targetVectorInSquare4 && currentHeadInSquare1) {
                angleChange = 360 - angleChange;
            }


            boolean reduceLeft = false;

            if (targetVector < currentHead) {
                reduceLeft = true;
            }

            if (targetVectorInSquare1 && currentHeadInSquare4 || targetVectorInSquare4 && currentHeadInSquare1 || this.target.getDirection() == RobotDirection.Backward) {
                reduceLeft = !reduceLeft;
            }


            double alpha = 90 - angleChange;
            double theta = Math.toRadians(angleChange * 2);
            double halfChord = chord / 2;
            double cosAlpha = Math.cos(Math.toRadians(alpha));
            double radius = 0;
            if (cosAlpha != 0) {
                radius = halfChord / cosAlpha;
            }


            double longArch = chord;
            double shortArch = chord;
            double speedRatio = 1;
            double lowSpeed = this.target.getTopSpeed();

            if (angleChange > 0) {
                //double centerArch = theta * radius;
                double wheelDistFromCenter = this.config.getWheelBaseSeparation() / 2;
                longArch = theta * (radius + wheelDistFromCenter);
                shortArch = theta * (radius - wheelDistFromCenter);

                //for speed, the ratio is greater as the spread between the motorized wheels is wider by 0.5 inch on each side
                double longArchMotor = longArch + BotMoveProfile.MOTOR_WHEEL_OFFSET;
                double shortArchMotor = shortArch - BotMoveProfile.MOTOR_WHEEL_OFFSET;
                speedRatio = shortArchMotor / longArchMotor;
                lowSpeed = this.target.getTopSpeed() * speedRatio;
            }


            double leftSpeed, rightSpeed;
            if (reduceLeft) {
                leftSpeed = lowSpeed;
                rightSpeed = this.target.getTopSpeed();
            } else {
                leftSpeed = this.target.getTopSpeed();
                rightSpeed = lowSpeed;
            }

            if (this.target.getDirection() == RobotDirection.Backward) {
                shortArch = -shortArch;
                longArch = -longArch;
            } else {
                rightSpeed = -rightSpeed;
                leftSpeed = -leftSpeed;
            }


            double distanceLong = longArch * YellowBot.COUNTS_PER_INCH_REV;
            double distanceShort = shortArch * YellowBot.COUNTS_PER_INCH_REV;

            boolean leftLong = true;
            double startingPointLong = 0, startingPointShort = 0;


            if (leftSpeed > rightSpeed) {
                startingPointLong = bot.getLeftOdometer();
                startingPointShort = bot.getRightOdometer();
            } else if (rightSpeed > leftSpeed) {
                leftLong = false;
                startingPointShort = bot.getLeftOdometer();
                startingPointLong = bot.getRightOdometer();
            }

            double averagePower = (Math.abs(rightSpeed) + Math.abs(leftSpeed)) / 2;
            averagePower = Math.round(averagePower * 10) / 10.0;
            double breakPoint = this.target.getMotorReduction().getBreakPoint(averagePower);

            double slowdownMarkLong = startingPointLong + (distanceLong - breakPoint);
            double slowdownMarkShort = startingPointShort + (distanceShort - breakPoint);

            double longTarget = startingPointLong + distanceLong;


            realSpeedLeft = leftSpeed;
            realSpeedRight = rightSpeed;
            this.leftLong = leftLong;
            this.slowdownMarkLong = slowdownMarkLong;
            this.slowdownMarkShort = slowdownMarkShort;
            this.longTarget = longTarget;

        }
        catch (Exception ex){
            ex.printStackTrace();
        }
    }


    public void stop(){ isRunning = false; }

    public double getX(){ return robotGlobalXCoordinatePosition; }


    public double getY(){ return robotGlobalYCoordinatePosition; }

    public double getXInches(){ return robotGlobalXCoordinatePosition/bot.COUNTS_PER_INCH_REV; }


    public double getYInches(){ return robotGlobalYCoordinatePosition/bot.COUNTS_PER_INCH_REV; }


    public double getOrientation(){ return Math.toDegrees(robotOrientationRadians) % 360; }


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

    public double getFrontCenterX() {
        return frontCenterX;
    }

    public void setFrontCenterX(double frontCenterX) {
        this.frontCenterX = frontCenterX;
    }

    public double getFrontCenterY() {
        return frontCenterY;
    }

    public void setFrontCenterY(double frontCenterY) {
        this.frontCenterY = frontCenterY;
    }

    public double getFrontCenterXInches() {
        return getFrontCenterX()/bot.COUNTS_PER_INCH_REV;
    }


    public double getFrontCenterYInches() {
        return getFrontCenterY()/bot.COUNTS_PER_INCH_REV;
    }

    public BotMoveRequest getTarget() {
        return target;
    }

    public void setTarget(BotMoveRequest target) {
        this.target = target;
    }


    public double getRealSpeedLeft() {
        return realSpeedLeft;
    }

    public double getRealSpeedRight() {
        return realSpeedRight;
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

    public double getInitialOrientation() {
        return initialOrientation;
    }

    public void setInitialOrientation(double initialOrientation) {
        this.initialOrientation = initialOrientation;
    }
}
