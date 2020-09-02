package org.firstinspires.ftc.teamcode.bots;

import android.graphics.Point;

import org.firstinspires.ftc.teamcode.calibration.BotCalibConfig;
import org.firstinspires.ftc.teamcode.calibration.MotorReductionBot;
import org.firstinspires.ftc.teamcode.odometry.RobotCoordinatePostiion;
import org.firstinspires.ftc.teamcode.skills.Geometry;

public class BotMoveProfile {
    private double realSpeedLeft = 0;
    private double realSpeedRight = 0;
    private double longTarget = 0;
    private double slowdownMarkLong = 0;
    private double slowdownMarkShort = 0;
    private boolean leftLong;
    private double speedRatio = 1;
    private double topSpeed = 0;
    private RobotDirection direction;
    private MotorReductionBot motorReduction = null;
    private BotMoveRequest target = new BotMoveRequest();
    private MoveStrategy strategy = MoveStrategy.Curve;
    private MoveStrategy nextStep = null;

    private double currentHead = 0;
    private double targetVector = 0;
    private double angleChange = 0;

    private double distanceRatio = 1;
    private double distance = 0;

    private Point start;
    private Point destination;
    private Point actual;

    public static double MOTOR_WHEEL_OFFSET = 1.25;

    public BotMoveProfile(){

    }

    public BotMoveProfile(MoveStrategy strategy){
        super();
        this.setStrategy(strategy);
    }

    @Override
    public String toString() {
        return String.format("Long Target: %.2f Direction: %s \nL:%.2f  R:%.2f\n Slowdownns: %.2f %.2f\nHead: %.2f Target: %.2f Change: %.2f\nFrom: %d %d\nTo: %d %d\nActual: %d %d\nSpeedR: %.2f Dist R: %.2f", longTarget, direction.name(), realSpeedLeft, realSpeedRight, slowdownMarkLong,slowdownMarkShort, currentHead, targetVector, angleChange, start.x, start.y, destination.x, destination.y, actual.x, actual.y, speedRatio, distanceRatio);
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

    public double getCurrentHead() {
        return currentHead;
    }

    public void setCurrentHead(double currentHead) {
        this.currentHead = currentHead;
    }

    public double getTargetVector() {
        return targetVector;
    }

    public void setTargetVector(double targetVector) {
        this.targetVector = targetVector;
    }

    public double getAngleChange() {
        return angleChange;
    }

    public void setAngleChange(double angleChange) {
        this.angleChange = angleChange;
    }

    public Point getStart() {
        return start;
    }

    public void setStart(Point start) {
        this.start = start;
    }

    public Point getDestination() {
        return destination;
    }

    public void setDestination(Point destination) {
        this.destination = destination;
    }

    public Point getActual() {
        return actual;
    }

    public void setActual(Point actual) {
        this.actual = actual;
    }

    public static BotMoveProfile bestRoute(OdoBot bot, double currentX, double currentY, Point target, RobotDirection direction, double topSpeed, MoveStrategy preferredStrategy, RobotCoordinatePostiion locator){
        double currentHead = locator.getOrientation();

        boolean clockwise = currentHead >= 0;
        if (!clockwise){
            currentHead = 360 + currentHead;
        }

        if (direction == RobotDirection.Backward) {
            currentHead = (currentHead + 180) % 360;
        }

        boolean currentHeadInSquare4 = currentHead >=270 && currentHead <= 360;
        boolean currentHeadInSquare1 = currentHead >=0 && currentHead <= 90;

        double distance = Geometry.getDistance(currentX, currentY, target.x, target.y);

        //determine the new heading to the target
        double distanceX = target.x - currentX;
        double distanceY = target.y - currentY;
        double targetVector = Math.toDegrees(Math.atan2(distanceY, distanceX));

        if (distanceY == 0){
            if (distanceX < 0){
                targetVector = 270;
            }
            else{
                targetVector = 90;
            }
        }
        else if (distanceX == 0){
            if(distanceY < 0){
                targetVector = 180;
            }
            else{
                targetVector = 0;
            }
        }
        else{
            //lower left
            if (distanceX < 0 && distanceY < 0){
                targetVector = (-targetVector) + 90;
            }
            //lower right
            if (distanceX > 0 && distanceY < 0){
                targetVector = (-targetVector) + 90;
            }
            //upper right
            if (distanceX > 0 && distanceY > 0){
                targetVector = 90 - targetVector;
            }
            //upper left
            if (distanceX < 0 && distanceY > 0){
                targetVector = 360 - (targetVector - 90);
            }
        }

        boolean targetVectorInSquare1 = targetVector >= 0 && targetVector <= 90;
        boolean targetVectorInSquare4 = targetVector >= 270 && targetVector <= 360;

        double realAngleChange = Geometry.getAngle(targetVector, currentHead);
        double angleChange = Math.abs(realAngleChange);

        if (preferredStrategy == MoveStrategy.Spin && angleChange > 10){
            return buildSpinProfile(realAngleChange, topSpeed, MoveStrategy.Curve);
        }

        if (preferredStrategy == MoveStrategy.Strafe){
            return buildStrafeProfile(bot.getCalibConfig(), realAngleChange, topSpeed, distance, MoveStrategy.Curve);
        }
        if (angleChange > 90){
            if (direction == RobotDirection.Optimal) {
                //better go backwards
                currentHead = (currentHead + 180) % 360;
                angleChange = Math.abs(Geometry.getAngle(targetVector, currentHead));
                direction = RobotDirection.Backward;
            }
            else {
                //spin
                bot.getTelemetry().addData("Route",  "Spin");
                return buildSpinProfile(realAngleChange, topSpeed, MoveStrategy.Curve);
            }
        }

        if (direction == RobotDirection.Optimal){
            direction = RobotDirection.Forward;
        }

        if (angleChange >= 42 && angleChange <= 48){
            //diag
            bot.getTelemetry().addData("Route",  "Diag");
            return new BotMoveProfile(MoveStrategy.Diag);
        }

        double chord = Math.sqrt(distanceX * distanceX + distanceY * distanceY);

        boolean reduceLeft = false;

        if (targetVector < currentHead) {
            reduceLeft = true;
        }

        if (targetVectorInSquare1 && currentHeadInSquare4 || targetVectorInSquare4 && currentHeadInSquare1 || direction == RobotDirection.Backward){
            reduceLeft = !reduceLeft;
        }

        bot.getTelemetry().addData("Target Vector", targetVector);
        bot.getTelemetry().addData("Angle Change", angleChange);



        //check radius.
        double radius = Geometry.getRadius(angleChange, chord);

        if ((reduceLeft && radius <= bot.getCalibConfig().getMinRadiusLeft()) ||
                (reduceLeft == false && radius <= bot.getCalibConfig().getMinRadiusRight())) {
            bot.getTelemetry().addData("Radius", "Too small. Cannot turn. Attempt to spin");
            return buildSpinProfile(realAngleChange, topSpeed, MoveStrategy.Curve);
        }

        BotMoveProfile profile = buildMoveProfile(bot, chord, topSpeed, radius, angleChange, reduceLeft, direction);
        //build request
        BotMoveRequest rq = new BotMoveRequest();
        rq.setTarget(target);
        rq.setTopSpeed(topSpeed);
        rq.setDirection(direction);
        rq.setMotorReduction(profile.getMotorReduction());
        profile.setTarget(rq);
        profile.setStart(new Point((int)locator.getXInches(), (int)locator.getYInches()));
        profile.setAngleChange(angleChange);
        profile.setCurrentHead(currentHead);
        profile.setTargetVector(targetVector);
        return profile;
    }

    public static BotMoveProfile buildMoveProfile(OdoBot bot, double chord, double topSpeed, double radius, double angleChange, boolean reduceLeft, RobotDirection direction){
        BotMoveProfile profile = new BotMoveProfile();

        double longArch = chord;
        double shortArch = chord;
        double speedRatio = 1;
        double lowSpeed = topSpeed;
        BotCalibConfig botConfig = bot.getCalibConfig();

        if (angleChange > 0) {
            double theta = Math.toRadians(angleChange * 2);
            //double centerArch = theta * radius;
            double wheelDistFromCenter = botConfig.getWheelBaseSeparation() / 2;
            longArch = theta * (radius + wheelDistFromCenter);
            shortArch = theta * (radius - wheelDistFromCenter);
            //for speed, the ratio is greater as the spread between the motorized wheels is wider by MOTOR_WHEEL_OFFSET on each side
            double longArchMotor = longArch + MOTOR_WHEEL_OFFSET;
            double shortArchMotor = shortArch - MOTOR_WHEEL_OFFSET;
            speedRatio = shortArchMotor / longArchMotor;
            lowSpeed = topSpeed * speedRatio;
        }

        profile.setSpeedRatio(speedRatio);
        profile.setDistanceRatio(shortArch/longArch);

        double leftSpeed, rightSpeed;
        if (reduceLeft) {
            leftSpeed = lowSpeed;
            rightSpeed = topSpeed;
        } else {
            leftSpeed = topSpeed;
            rightSpeed = lowSpeed;
        }

        if (direction == RobotDirection.Backward){
            shortArch = -shortArch;
            longArch = -longArch;
        }
        else{
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

        double averagePower = (Math.abs(rightSpeed) + Math.abs(leftSpeed))/2;
        averagePower = Math.round(averagePower*10)/10.0;

        MotorReductionBot mr = botConfig.getMoveMRForward();
        if (direction == RobotDirection.Backward) {
            mr = botConfig.getMoveMRBack();
        }
        double breakPoint = mr.getBreakPoint(averagePower);

        double slowdownMarkLong = startingPointLong + (distanceLong - breakPoint);
        double slowdownMarkShort = startingPointShort + (distanceShort - breakPoint);

        double longTarget = startingPointLong + distanceLong;


        profile.setLeftLong(leftLong);
        profile.setSlowdownMarkLong(slowdownMarkLong);
        profile.setSlowdownMarkShort(slowdownMarkShort);
        profile.setLongTarget(longTarget);
        profile.setRealSpeedLeft(leftSpeed);
        profile.setRealSpeedRight(rightSpeed);
        profile.setMotorReduction(mr);
        profile.setDirection(direction);
        return profile;
    }

    private static BotMoveProfile buildSpinProfile(double angleChange, double topSpeed, MoveStrategy next){
        BotMoveProfile profile = new BotMoveProfile();
        profile.setAngleChange(angleChange);
        profile.setStrategy(MoveStrategy.Spin);
        profile.setTopSpeed(topSpeed);
        profile.setNextStep(next);
        return profile;
    }

    private static BotMoveProfile buildStrafeProfile(BotCalibConfig botConfig, double angleChange, double topSpeed, double distance, MoveStrategy next){
        BotMoveProfile profile = new BotMoveProfile();
        boolean left = angleChange > 0;
        if (left){
            profile.setMotorReduction(botConfig.getStrafeLeftReduction());
        }
        else{
            distance = -distance;
            profile.setMotorReduction(botConfig.getStrafeRightReduction());
        }
        profile.setDistance(distance);
        profile.setAngleChange(angleChange);
        profile.setStrategy(MoveStrategy.Strafe);
        profile.setTopSpeed(topSpeed);
        profile.setNextStep(next);
        return profile;
    }

    public MoveStrategy getStrategy() {
        return strategy;
    }

    public void setStrategy(MoveStrategy strategy) {
        this.strategy = strategy;
    }

    public double getDistanceRatio() {
        return distanceRatio;
    }

    public void setDistanceRatio(double distanceRatio) {
        this.distanceRatio = distanceRatio;
    }

    public double getTopSpeed() {
        return topSpeed;
    }

    public void setTopSpeed(double topSpeed) {
        this.topSpeed = topSpeed;
    }

    public MoveStrategy getNextStep() {
        return nextStep;
    }

    public void setNextStep(MoveStrategy nextStep) {
        this.nextStep = nextStep;
    }

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }
}
