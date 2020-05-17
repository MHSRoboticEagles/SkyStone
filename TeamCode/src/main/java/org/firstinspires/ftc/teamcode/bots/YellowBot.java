package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.calibration.BotCalibConfig;
import org.firstinspires.ftc.teamcode.skills.Gyro;
import org.firstinspires.ftc.teamcode.skills.Gyroscope;
import org.firstinspires.ftc.teamcode.skills.Led;

import java.io.File;

public class YellowBot extends UberBot {
    public static double CALIB_SPEED = 0.5;
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    HardwareMap hwMap  =  null;
    private Telemetry telemetry;

    private Gyroscope gyro = null;

    private Led led = null;

    private LinearOpMode owner = null;

    static final double     COUNTS_PER_MOTOR_HD    = 560 ;    // Rev HD motor
    static final double     REV_TBORE    = 8192 ;    // Rev HD motor
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP. was 2 in the sample
    static final double     WHEEL_DIAMETER_INCHES   = 2.0 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH_REV     = (REV_TBORE * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);  //1303.79729

    public static final double ROBOT_LENGTH_X = 17.25;
    public static final double ROBOT_LENGTH_Y = 17.5;

    public static final double ROBOT_CENTER_X = 8.25;
    public static final double ROBOT_CENTER_Y = 8.25;

    public static final double ODO_WHEEL_DISTANCE = 12.25;

    public static final double ROBOT_FRONT_X = 9;
    public static final double ROBOT_FORNT_Y = 9;

    private final double LEFT_REDUCTION = 1;//0.955;

    //move to individual op mode
    public static final double START_X = 48;
    public static final double START_Y = 0;

    private static double currentX = 0;
    private static double currentY = 0;

    private double leftSpinByDegree = 0;
    private double rightSpinByDegree = 0;
    private double wheelBaseSeparation = 0;
    private double horizontalTicksDegree = 0;
    private double minRadiusLeft = 0;
    private double minRadiusRight = 0;


    public YellowBot() {

    }

    public void init(LinearOpMode owner, HardwareMap hw, Telemetry t)throws Exception {
        this.owner = owner;
        this.hwMap = hw;
        this.telemetry = t;
        try {
            // Define and Initialize Motors
            frontLeft = hwMap.get(DcMotor.class, "frontLeft");
            frontRight = hwMap.get(DcMotor.class, "frontRight");
            backLeft = hwMap.get(DcMotor.class, "backLeft");
            backRight = hwMap.get(DcMotor.class, "backRight");

            resetEncoders();


            if (backLeft != null) {
                backLeft.setDirection(DcMotor.Direction.FORWARD);
                backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (backRight != null) {
                backRight.setDirection(DcMotor.Direction.REVERSE);
                backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (frontLeft != null) {
                frontLeft.setDirection(DcMotor.Direction.FORWARD);
                frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (frontRight != null) {
                frontRight.setDirection(DcMotor.Direction.REVERSE);
                frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            stop();
        }
        catch (Exception ex){
            //issues accessing drive resources
            throw new Exception("Issues accessing one of drive motors. Check the controller config", ex);
        }
    }

    protected void resetEncoders(){
        if (frontLeft != null && frontRight!= null && backLeft != null && backRight != null)
        {
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void stop (){
        if (frontLeft != null && frontRight!= null && backLeft != null && backRight != null) {

            // Set all motors to zero power
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }

    public double getOdemeteReading(boolean left){
        if (left){
            return frontLeft.getCurrentPosition();
        }
        else {
            return frontRight.getCurrentPosition();
        }

    }

    public double getLeftOdemeter(){
        return   frontLeft.getCurrentPosition();

    }

    public double getRightOdemeter(){
        return   frontRight.getCurrentPosition();

    }

    public double getHorizontalOdemeter(){
        return  backLeft.getCurrentPosition();

    }

    public void move(double drive, double turn){
        if (frontLeft != null && frontRight!= null && backLeft != null && backRight != null) {
            double rightPower = Range.clip(drive + turn, -1.0, 1.0);
            double leftPower = Range.clip(drive - turn, -1.0, 1.0);

            //create dead zone for bad joysticks
            if (Math.abs(rightPower) < 0.02){
                rightPower = 0;
            }

            if (Math.abs(leftPower) < 0.02){
                leftPower = 0;
            }

            //apply logarythmic adjustment

            rightPower = rightPower * 100 / 110;
            rightPower = rightPower * rightPower * rightPower;

            leftPower = leftPower * 100 / 110;
            leftPower = leftPower * leftPower * leftPower;

            this.frontLeft.setPower(leftPower);
            this.frontRight.setPower(rightPower);
            this.backLeft.setPower(leftPower);
            this.backRight.setPower(rightPower);

            telemetry.addData("Left Speed", leftPower);
            telemetry.addData("Right Speed", rightPower);
            telemetry.addData("Odo", "Left from %7d", frontLeft.getCurrentPosition());
            telemetry.addData("Odo", "Right from %7d", frontRight.getCurrentPosition());
        }
    }

    public void moveTo(double leftspeed, double rightspeed, double inches){
        if (frontLeft != null && frontRight!= null && backLeft != null && backRight != null) {
            double rightPower = rightspeed;
            double leftPower = leftspeed*LEFT_REDUCTION;
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            boolean forward = inches > 0;

            //reverse speed
            if (forward){
                rightPower = -rightPower;
                leftPower = -leftPower;
            }

            double distance = inches * COUNTS_PER_INCH_REV;

            double startingPoint = this.getLeftOdemeter();

            double leftTarget = this.getLeftOdemeter() + distance;

            double cutOff = ROBOT_LENGTH_Y* COUNTS_PER_INCH_REV;
            //at top speed the slow-down should start at a 23% greater distance than at half speed.
            cutOff = cutOff + ROBOT_LENGTH_Y*0.5* COUNTS_PER_INCH_REV;

            double slowdownDistance = Math.abs(distance) - cutOff;

            double slowdownMark = 0;

            if (slowdownDistance < 0){
                slowdownMark = distance * 0.1;
                if (forward){
                    rightPower = -CALIB_SPEED;
                }
                else{
                    rightPower = CALIB_SPEED;
                }
            }
            else{
                if (forward){
                    slowdownMark = startingPoint + slowdownDistance;
                }
                else{
                    slowdownMark = startingPoint - slowdownDistance;
                }
            }


            double minSpeed = 0.2;

            double speedDropStep = 0.1;


            double originalRight = rightPower;


            double speedIncrement = 0.05;
            if (forward){
                speedIncrement = -speedIncrement;
            }
            leftPower = 0;
            rightPower = 0;

            double realSpeedLeft = leftPower;
            double realSpeedRight = rightPower;

            this.frontLeft.setPower(leftPower);
            this.frontRight.setPower(rightPower);
            this.backLeft.setPower(leftPower);
            this.backRight.setPower(rightPower);

            boolean stop = false;
            int step = 0;
            while (!stop && owner.opModeIsActive()) {
                double leftreading = this.getLeftOdemeter();
                stop =  (forward && leftreading >= leftTarget) ||
                        (forward == false && leftreading <= leftTarget);
                if (stop){
                    break;
                }
                if ((forward && leftreading >= slowdownMark) ||
                        (forward == false && leftreading <= slowdownMark)){
                    step++;

                    if (forward) {
                        rightPower = realSpeedRight + speedDropStep * step;
                        leftPower = realSpeedLeft + speedDropStep * step;
                        if (rightPower >= -minSpeed || leftPower >= -minSpeed){
                            leftPower = -minSpeed;
                            rightPower = -minSpeed;
                        }
                    }
                    else{
                        rightPower = realSpeedRight - speedDropStep * step;
                        leftPower = realSpeedLeft - speedDropStep * step;
                        if (rightPower <=minSpeed || leftPower <= minSpeed){
                            leftPower = minSpeed;
                            rightPower = minSpeed;
                        }
                    }
                }
                else{
                    //acceleration
                    if ((forward && rightPower + speedIncrement >= originalRight) ||
                            (!forward && rightPower + speedIncrement <= originalRight)) {
                        rightPower = rightPower + speedIncrement;
                        leftPower = rightPower * LEFT_REDUCTION;
                        realSpeedLeft = leftPower;
                        realSpeedRight = rightPower;
                    }
                }

                this.frontLeft.setPower(leftPower);
                this.frontRight.setPower(rightPower);
                this.backLeft.setPower(leftPower);
                this.backRight.setPower(rightPower);
            }

            this.stop();
        }
    }

    public void spin(double desiredHeading, double speed){
        speed = Math.abs(speed);
        double currentHead = this.getGyroHeading();
        boolean spinLeft = false;
        if (desiredHeading > currentHead){
            spinLeft = true;
        }

        double leftDesiredSpeed = speed;
        double rightDesiredSpeed = speed;

        if (spinLeft){
            rightDesiredSpeed = -rightDesiredSpeed;
        }
        else{
            leftDesiredSpeed = -leftDesiredSpeed;
        }

        double archDegrees = Math.abs(desiredHeading - currentHead);
        double leftDistance = archDegrees * leftSpinByDegree;
        double rightDistance = archDegrees * rightSpinByDegree;



        double minDistance = rightDistance;
        boolean leftOdometer = false;
        if (rightDistance > leftDistance) {
            minDistance = leftDistance;
            leftOdometer = true;
        }

        double startingPoint = this.getOdemeteReading(leftOdometer);

        double slowdownDistance = minDistance * 0.4;

        double slowdownMark = 0;

        boolean targetIncrement = true;
        double target = 0;
        if ((spinLeft && leftOdometer) || (!spinLeft && !leftOdometer)){
            target = startingPoint - minDistance;
            slowdownMark = startingPoint - slowdownDistance;
            targetIncrement = false;
        }

        if ((spinLeft && !leftOdometer) || (!spinLeft && leftOdometer)){
            target = startingPoint + minDistance;
            slowdownMark = startingPoint + slowdownDistance;
            targetIncrement = true;
        }


        double leftPower = 0;
        double rightPower = 0;

        double realSpeedLeft = leftPower;
        double realSpeedRight = rightPower;

        double speedIncrement = 0.05;

        boolean stop = false;
        int step = 0;
        double minSpeed = 0.1;

        double speedDropStep = 0.1;
        while (!stop && this.owner.opModeIsActive()){
            double currentReading = this.getOdemeteReading(leftOdometer);
            if ((targetIncrement && currentReading >= target) ||
                    (!targetIncrement && currentReading <= target)){
                stop = true;
            }
            if (!stop) {
                //slow down
                if ((targetIncrement && currentReading >= slowdownMark) ||
                        (!targetIncrement && currentReading <= slowdownMark)){
                    step++;

                    if (spinLeft) {
                        rightPower = realSpeedRight + speedDropStep * step;
                        leftPower = realSpeedLeft - speedDropStep * step;
                        if (rightPower >= -minSpeed || leftPower <= minSpeed){
                            leftPower = minSpeed;
                            rightPower = -minSpeed;
                        }
                    }
                    else{
                        rightPower = realSpeedRight - speedDropStep * step;
                        leftPower = realSpeedLeft + speedDropStep * step;
                        if (rightPower <=minSpeed || leftPower >= -minSpeed){
                            leftPower = -minSpeed;
                            rightPower = minSpeed;
                        }
                    }
                }
                else {
                    //accelerate
                    if ((spinLeft && leftPower + speedIncrement <= leftDesiredSpeed) ||
                            (!spinLeft && rightPower + speedIncrement <= rightDesiredSpeed)) {
                        if (spinLeft) {
                            leftPower = leftPower + speedIncrement;
                            rightPower = -leftPower;
                        } else {
                            rightPower = rightPower + speedIncrement;
                            leftPower = -rightPower;
                        }
                        realSpeedLeft = leftPower;
                        realSpeedRight = rightPower;
                    }
                }
            }
            this.frontLeft.setPower(leftPower);
            this.frontRight.setPower(rightPower);
            this.backLeft.setPower(leftPower);
            this.backRight.setPower(rightPower);
        }

        this.stop();
    }

    public void spinH(double desiredHeading, double speed){
        speed = Math.abs(speed);
        double currentHead = this.getGyroHeading();
        boolean spinLeft = false;
        if (desiredHeading > currentHead){
            spinLeft = true;
        }

        double leftDesiredSpeed = speed;
        double rightDesiredSpeed = speed;

        if (spinLeft){
            rightDesiredSpeed = -rightDesiredSpeed;
        }
        else{
            leftDesiredSpeed = -leftDesiredSpeed;
        }

        double archDegrees = Math.abs(desiredHeading - currentHead);
        double horDistance = archDegrees * horizontalTicksDegree;



        double startingPoint = this.getHorizontalOdemeter();

        double slowdownDistance = horDistance * 0.4;

        double slowdownMark = 0;

        boolean targetIncrement = true;
        double target = 0;
        if (spinLeft){
            target = startingPoint - horDistance;
            slowdownMark = startingPoint - slowdownDistance;
            targetIncrement = false;
        }

        if (!spinLeft){
            target = startingPoint + horDistance;
            slowdownMark = startingPoint + slowdownDistance;
            targetIncrement = true;
        }


        double leftPower = 0;
        double rightPower = 0;

        double realSpeedLeft = leftPower;
        double realSpeedRight = rightPower;

        double speedIncrement = 0.05;

        boolean stop = false;
        int step = 0;
        double minSpeed = 0.1;

        double speedDropStep = 0.1;
        while (!stop && this.owner.opModeIsActive()){
            double currentReading = this.getHorizontalOdemeter();
            if ((targetIncrement && currentReading >= target) ||
                    (!targetIncrement && currentReading <= target)){
                stop = true;
            }
            if (!stop) {
                //slow down
                if ((targetIncrement && currentReading >= slowdownMark) ||
                        (!targetIncrement && currentReading <= slowdownMark)){
                    step++;

                    if (spinLeft) {
                        rightPower = realSpeedRight + speedDropStep * step;
                        leftPower = realSpeedLeft - speedDropStep * step;
                        if (rightPower >= -minSpeed || leftPower <= minSpeed){
                            leftPower = minSpeed;
                            rightPower = -minSpeed;
                        }
                    }
                    else{
                        rightPower = realSpeedRight - speedDropStep * step;
                        leftPower = realSpeedLeft + speedDropStep * step;
                        if (rightPower <=minSpeed || leftPower >= -minSpeed){
                            leftPower = -minSpeed;
                            rightPower = minSpeed;
                        }
                    }
                }
                else {
                    //accelerate
                    if ((spinLeft && leftPower + speedIncrement <= leftDesiredSpeed) ||
                            (!spinLeft && rightPower + speedIncrement <= rightDesiredSpeed)) {
                        if (spinLeft) {
                            leftPower = leftPower + speedIncrement;
                            rightPower = -leftPower;
                        } else {
                            rightPower = rightPower + speedIncrement;
                            leftPower = -rightPower;
                        }
                        realSpeedLeft = leftPower;
                        realSpeedRight = rightPower;
                    }
                }
            }
            this.frontLeft.setPower(leftPower);
            this.frontRight.setPower(rightPower);
            this.backLeft.setPower(leftPower);
            this.backRight.setPower(rightPower);
        }

        this.stop();
    }

    public double getLeftTarget(double inches){
        return this.getLeftOdemeter() + inches * COUNTS_PER_INCH_REV;
    }

    public double getRightTarget(double inches){
        return this.getRightOdemeter() + inches * COUNTS_PER_INCH_REV;
    }

    public void moveToCoordinate(double X, double Y, double targetX, double targetY, RobotDirection direction, double desiredHead, double topSpeed){
        //X and Y are front center of the robot
        double currentX = X;
        double currentY = Y;
        double currentHead = this.getGyroHeading();


        //determine the new heading to the target
        double distanceX = Math.abs(targetX - currentX);
        double distanceY = Math.abs(targetY - currentY);
        double angle = Math.toDegrees(Math.atan(distanceX/distanceY));
        double chord = Math.sqrt(distanceX*distanceX + distanceY * distanceY);

        telemetry.addData("Chord", chord);

        //find the direction of the new target
        double targetVector = angle;
        if (targetY < currentY){
            targetVector = 180 - angle;
        }

        if (targetX > currentX){
            targetVector = - targetVector;
        }

        telemetry.addData("Target Vector", targetVector);

        double angleChange = Math.abs(targetVector - currentHead);
        double sign = -currentHead/Math.abs(currentHead);
        if (angleChange > 90){
            //backwards is faster
            if (direction == RobotDirection.Backward){
                currentHead = (180 - Math.abs(currentHead))*sign;
            }
            else{
                //spin
                spinH(targetVector, topSpeed);
                //todo: make more precise
                currentHead = targetVector;
            }
            angleChange = Math.abs(targetVector - currentHead);
        }

        telemetry.addData("CurrentHead", currentHead);
        telemetry.addData("AngleChange", angleChange);

        boolean turnLeft = false;

        if (targetVector > currentHead){
            turnLeft = true;
        }
        telemetry.addData("turnLeft", turnLeft);

        double alpha = 90 - angleChange;
        double theta = Math.toRadians(angleChange * 2);
        double halfChord = chord/2;
        double radius = halfChord/Math.cos(Math.toRadians(alpha));

        if ((turnLeft && radius <= this.minRadiusLeft) ||
                (turnLeft == false && radius <=this.minRadiusRight)){
            telemetry.addData("Radius", "Too small. Cannot turn");
        }else {
            //double centerArch = theta * radius;
            double wheelDistFromCenter = this.wheelBaseSeparation / 2;
            double longArch = theta * (radius + wheelDistFromCenter) * COUNTS_PER_INCH_REV;
            double shortArch = theta * (radius - wheelDistFromCenter) * COUNTS_PER_INCH_REV;
            double speedRatio = shortArch / longArch;
            double lowSpeed = topSpeed * speedRatio;

            telemetry.addData("Radius", radius);
            telemetry.addData("Theta", theta);
            telemetry.addData("longArch", longArch);
            telemetry.addData("shortArch", shortArch);
            if (turnLeft) {
                telemetry.addData("right speed", topSpeed);
                telemetry.addData("left speed", lowSpeed);
            } else {
                telemetry.addData("left speed", topSpeed);
                telemetry.addData("right speed", lowSpeed);
            }
        }

        telemetry.update();

    }

    public void spinLeft(double speed, boolean forward){
        if (frontLeft != null && frontRight!= null && backLeft != null && backRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);

            if (forward){
                power = -power;
            }

            this.frontLeft.setPower(-power);
            this.frontRight.setPower(power);
            this.backLeft.setPower(-power);
            this.backRight.setPower(power);
            telemetry.addData("Odo", "Left from %7d", frontLeft.getCurrentPosition());
            telemetry.addData("Odo", "Right from %7d", frontRight.getCurrentPosition());
        }
    }

    public void spinRight(double speed, boolean forward){
        if (frontLeft != null && frontRight!= null && backLeft != null && backRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);

            if (forward){
                power = -power;
            }

            this.frontLeft.setPower(power);
            this.frontRight.setPower(-power);
            this.backLeft.setPower(power);
            this.backRight.setPower(-power);
            telemetry.addData("Odo", "Left from %7d", frontLeft.getCurrentPosition());
            telemetry.addData("Odo", "Right from %7d", frontRight.getCurrentPosition());
        }
    }

    public void spinLeftDegrees(double speed, double degrees, boolean forward){
        File leftSpinPerDegFile = AppUtil.getInstance().getSettingsFile("leftSpinPerDeg.txt");
        File rightSpinPerDegFile = AppUtil.getInstance().getSettingsFile("rightSpinPerDeg.txt");

        double leftPerDegree = Double.parseDouble(ReadWriteFile.readFile(leftSpinPerDegFile).trim());
        double rightPerDegree = Double.parseDouble(ReadWriteFile.readFile(rightSpinPerDegFile).trim());

        double targetPos = this.getLeftOdemeter() + leftPerDegree * degrees;

        double power = Range.clip(speed, -1.0, 1.0);

        if (forward){
            power = -power;
        }

        this.frontLeft.setPower(-power);
        this.frontRight.setPower(power);
        this.backLeft.setPower(-power);
        this.backRight.setPower(power);
        while (this.getLeftOdemeter() > targetPos && owner.opModeIsActive()){

        }

        this.stop();
    }


    public void turnLeft(double speed, boolean forward){
        if (frontLeft != null && frontRight!= null && backLeft != null && backRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);

            if (forward){
                power = -power;
            }

            this.frontRight.setPower(power);
            this.backRight.setPower(power);
            telemetry.addData("Odo", "Left from %7d", frontLeft.getCurrentPosition());
            telemetry.addData("Odo", "Right from %7d", frontRight.getCurrentPosition());
        }
    }

    public void turnRight(double speed, boolean forward){
        if (frontLeft != null && frontRight!= null && backLeft != null && backRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);

            if (forward == true){
                power = -power;
            }

            this.frontLeft.setPower(power);
            this.backLeft.setPower(power);
            telemetry.addData("Odo", "Left from %7d", frontLeft.getCurrentPosition());
            telemetry.addData("Odo", "Right from %7d", frontRight.getCurrentPosition());
        }
    }

    public void strafeLeft(double speed){
        if (backLeft != null && backRight!= null && frontLeft != null && frontRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);
            power = power * power * power;
            this.backLeft.setPower(power);
            this.backRight.setPower(-power);
            this.frontLeft.setPower(-power);
            this.frontRight.setPower(power);
            telemetry.addData("Motors", "Front: %.0f", power);
            telemetry.addData("Motors", "Back: %.0f", power);
        }
    }

    public void strafeRight(double speed){
        if (backLeft != null && backRight!= null && frontLeft != null && frontRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);
            power = power * power * power;
            this.backLeft.setPower(-power);
            this.backRight.setPower(power);
            this.frontLeft.setPower(power);
            this.frontRight.setPower(-power);
            telemetry.addData("Motors", "Front: %.0f", power);
            telemetry.addData("Motors", "Back: %.0f", power);
        }
    }

    public double strafeTo(double speed, double inches, boolean left, double reduction) {
        double currentPos = this.getHorizontalOdemeter();
        double distance = inches * COUNTS_PER_INCH_REV;


        double overage = 0;

        if (left == false){
            distance = -distance;
        }

        double target = currentPos + distance;

        boolean stop = false;

        while (!stop && this.owner.opModeIsActive()){
            currentPos = this.getHorizontalOdemeter();
            if((left && currentPos >= target) || (left == false && currentPos <= target)){
                stop = true;
            }

            if (left){
                this.backLeft.setPower(-speed);
                this.backRight.setPower(speed);
                this.frontLeft.setPower(speed*reduction);
                this.frontRight.setPower(-speed*reduction);
            }
            else{
                this.backLeft.setPower(speed*reduction);
                this.backRight.setPower(-speed);
                this.frontLeft.setPower(-speed*reduction);
                this.frontRight.setPower(speed);
            }
        }

        stop();
        double newPos = this.getHorizontalOdemeter();
        double diff = Math.abs(newPos - target);
        overage = diff/distance*100;
        return overage;
    }


    public void diagLeft(double speed){
        if (backLeft != null && backRight!= null && frontLeft != null && frontRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);
            this.frontLeft.setPower(power);
            this.backRight.setPower(power);

            this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            this.backLeft.setPower(0);
            this.frontRight.setPower(0);
        }
    }

    public void diagRight(double speed){
        if (backLeft != null && backRight!= null && frontLeft != null && frontRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);

            this.backLeft.setPower(power);
            this.frontRight.setPower(power);

            this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            this.frontLeft.setPower(0);
            this.backRight.setPower(0);
        }
    }

    public void diagTo(double speed, double diagInches, boolean left){
        if (backLeft != null && backRight!= null && frontLeft != null && frontRight != null) {
            if (left) {
                double power = speed;

                if (diagInches > 0){
                    power = -power;
                }

                double leftOdoStart = getLeftOdemeter();
                double rightOdoStart = getRightOdemeter();
                double horOdoStart = getHorizontalOdemeter();

                double distance = diagInches * COUNTS_PER_INCH_REV;

                double hyp = 0;
                boolean stop = false;
                double angle = 0;
                double horDist = 0;
                double catet = 0;


                while(!stop && owner.opModeIsActive()){
                    double leftOdo = getLeftOdemeter();
                    double rightOdo = getRightOdemeter();
                    double horOdo = getHorizontalOdemeter();
                    double leftDist = leftOdo - leftOdoStart;
                    double rightDist = rightOdo - rightOdoStart;
                    horDist = horOdo - horOdoStart;
                    catet = leftDist > rightDist ? leftDist : rightDist;
                    hyp = Math.sqrt(horDist*horDist + catet * catet);
                    if (hyp >= distance){
                        angle = Math.toDegrees(Math.atan(catet/horDist));
                        break;
                    }

                    if (!left) {
                        this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                        this.frontLeft.setPower(power);
                        this.backRight.setPower(power);

                        this.backLeft.setPower(0);
                        this.frontRight.setPower(0);
                    }
                    else{
                        this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                        this.backLeft.setPower(power);
                        this.frontRight.setPower(power);


                        this.frontLeft.setPower(0);
                        this.backRight.setPower(0);
                    }
                }

                telemetry.addData("Angle", angle);
                telemetry.addData("Hor", horDist/COUNTS_PER_INCH_REV);
                telemetry.addData("Vert", catet/COUNTS_PER_INCH_REV);
                telemetry.addData("Hyp", catet/COUNTS_PER_INCH_REV);
                telemetry.update();
            }
            this.stop();
        }
    }


    public void initCalibData() throws Exception {
        File calibFile = AppUtil.getInstance().getSettingsFile(BotCalibConfig.BOT_CALIB_CONFIG);
        if (calibFile.exists()){
            String data = ReadWriteFile.readFile(calibFile);
            BotCalibConfig config = BotCalibConfig.deserialize(data);
            this.leftSpinByDegree = config.getLeftTickPerDegree();
            this.rightSpinByDegree = config.getRightTickPerDegree();
            this.wheelBaseSeparation = Math.abs(config.getWheelBaseSeparation());
            this.horizontalTicksDegree = config.getHorizontalTicksDegree();
            this.minRadiusLeft = config.getMinRadiusLeft();
            this.minRadiusRight = config.getMinRadiusRight();
            telemetry.addData("leftSpinByDegree", leftSpinByDegree);
            telemetry.addData("rightSpinByDegree", rightSpinByDegree);
            telemetry.addData("wheel separation", wheelBaseSeparation);
            telemetry.addData("horizontalTicksDegree", horizontalTicksDegree);
            telemetry.addData("minRadiusLeft", minRadiusLeft);
            telemetry.addData("minRadiusRight", minRadiusRight);
            telemetry.update();
            if (config == null){
                throw new Exception("Calibration data does not exist. Run Spin calibration first");
            }
        }
        else{
            throw new Exception("Calibration data does not exist. Run Spin calibration first");
        }
    }

    public BotCalibConfig getCalibConfig(){
        BotCalibConfig config = null;
        File calibFile = AppUtil.getInstance().getSettingsFile(BotCalibConfig.BOT_CALIB_CONFIG);
        if (calibFile.exists()) {
            String data = ReadWriteFile.readFile(calibFile);
            config = BotCalibConfig.deserialize(data);
        }
        return config;
    }

    public File getCalibConfigFile(){
        return AppUtil.getInstance().getSettingsFile(BotCalibConfig.BOT_CALIB_CONFIG);
    }


    ///gyroscope
    public void initGyro(){
        if (this.gyro == null){
            this.gyro = new Gyroscope();
        }
        File calibFile = AppUtil.getInstance().getSettingsFile(gyro.CALIB_FILE);
        if (calibFile.exists()) {
            this.gyro.init(this.hwMap, this.telemetry, false);
        }
        else{
            this.gyro.init(this.hwMap, this.telemetry, true);
            this.gyro.calibrate();
        }
    }

    public void calibrateGyro(){
        if (this.gyro == null){
            this.gyro = new Gyroscope();
        }

        this.gyro.init(this.hwMap, this.telemetry, true);
        this.gyro.calibrate();

    }

    public double getGyroHeading(){
        if(this.gyro != null){
            return this.gyro.getHeading();
        }

        return -666;
    }


    //leds

    public Led getLights(){
        if (led == null){
            led = new Led();
            led.init(hwMap);
        }
        return led;
    }


}
