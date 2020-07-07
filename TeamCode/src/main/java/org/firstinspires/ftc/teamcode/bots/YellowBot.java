package org.firstinspires.ftc.teamcode.bots;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.calibration.BotCalibConfig;
import org.firstinspires.ftc.teamcode.calibration.DiagCalibConfig;
import org.firstinspires.ftc.teamcode.calibration.MotorReductionBot;
import org.firstinspires.ftc.teamcode.odometry.RobotCoordinatePostiion;
import org.firstinspires.ftc.teamcode.skills.Geometry;
import org.firstinspires.ftc.teamcode.skills.Gyroscope;
import org.firstinspires.ftc.teamcode.skills.Led;

import java.io.File;

public class YellowBot {
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


    //move to individual op mode
    public static final double START_X = 48;
    public static final double START_Y = 0;

    private static double currentX = 0;
    private static double currentY = 0;


    private BotCalibConfig botConfig;


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
            double leftPower = leftspeed;
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
                        leftPower = rightPower;
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

    public RobotMovement moveToCalib(double leftspeed, double rightspeed, double inches, MotorReductionBot mr, double breakPoint, Led led){
        RobotMovement stats = new RobotMovement();
        if (frontLeft != null && frontRight!= null && backLeft != null && backRight != null) {
            double rightPower = rightspeed;
            double leftPower = leftspeed;
            stats.setMotorPower(Math.abs(leftPower));
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
            else{
                breakPoint = -breakPoint;
            }

            double distance = inches * COUNTS_PER_INCH_REV;

            double startingPoint = this.getLeftOdemeter();


            double slowdownMark = startingPoint + (distance - breakPoint);

            double leftTarget = startingPoint + distance;


            double minSpeed = 0.01;

            double speedDropStep = 0.05;

            double originalRight = rightPower;
            double originalLeft = leftPower;


            double speedIncrement = 0.05;
            if (forward){
                speedIncrement = -speedIncrement;
            }
            leftPower = 0;
            rightPower = 0;

            double realSpeedLeft = leftPower;
            double realSpeedRight = rightPower;

            boolean fullSpeedReached = false;

            stats.startAccelerateTimer(startingPoint);

            boolean stop = false;
            boolean slowDown = false;
            int step = 0;
            while (!stop && owner.opModeIsActive()) {
                double leftreading = this.getLeftOdemeter();
                if ((forward && leftreading >= slowdownMark) ||
                        (forward == false && leftreading <= slowdownMark)){

                    if (!slowDown) {
                        if (fullSpeedReached) {
                            fullSpeedReached = false;
                            stats.stopFullSpeedTimer(leftreading);
                        } else {
                            stats.stopAccelerateTimer(leftreading);
                        }
                        stats.startSlowDownTimer(leftreading, slowdownMark);
                        slowDown = true;
                    }
                    step++;
                    if (Math.abs(leftPower) <= Math.abs(minSpeed) || Math.abs(rightPower) <= Math.abs(minSpeed) ){
                        stop =  (forward && leftreading >= leftTarget) ||
                        (forward == false && leftreading <= leftTarget);
                        if (stop){
                            break;
                        }
                    }

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
                        leftPower = rightPower;
                        realSpeedLeft = leftPower;
                        realSpeedRight = rightPower;
                    }
                    else{
                        //full speed
                        if (!fullSpeedReached){
                            fullSpeedReached = true;
                            stats.stopAccelerateTimer(leftreading);
                            stats.startFullSpeedTimer(leftreading);
                        }
                    }
                }


                this.frontLeft.setPower(leftPower * mr.getLF());
                this.frontRight.setPower(rightPower * mr.getRF());
                this.backLeft.setPower(leftPower * mr.getLB());
                this.backRight.setPower(rightPower * mr.getRB());

            }
            stats.stopSlowdownTimer(this.getLeftOdemeter());

            stats.computeTotals(this.getLeftOdemeter());

            this.stop();
        }
        return stats;
    }

    public void moveCurveCalib(double leftspeed, double rightspeed, double lowSpeedReduction, double inchesShort, double inchesLong, RobotDirection direction, RobotCoordinatePostiion locator){
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {
            MotorReductionBot mr;
            mr = this.getCalibConfig().getMoveMRForward();
            if (direction == RobotDirection.Backward) {
                mr = this.getCalibConfig().getMoveMRBack();
            }

            double rightPower = rightspeed;
            double leftPower = leftspeed;
            double startPower = 0;

            double averagePower = (Math.abs(rightPower) + Math.abs(leftPower))/2;
            averagePower = Math.round(averagePower*10)/10.0;

            double breakPoint = mr.getBreakPoint(averagePower);

            boolean leftLong = true;
            double startingPointLong = 0, startingPointShort = 0;

            double speedIncrementLeft = 0.05;
            double speedIncrementRight = speedIncrementLeft;

            if (leftPower > rightPower) {
                startingPointLong = this.getLeftOdemeter();
                startingPointShort = this.getRightOdemeter();
                speedIncrementRight = speedIncrementLeft * lowSpeedReduction;
            } else if (rightPower > leftPower) {
                leftLong = false;
                startingPointShort = this.getLeftOdemeter();
                startingPointLong = this.getRightOdemeter();
                speedIncrementLeft = speedIncrementRight * lowSpeedReduction;
            }


            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            boolean forward = direction == RobotDirection.Forward;

            //reverse speed
            if (forward) {
                rightPower = -rightPower;
                leftPower = -leftPower;
                startPower = -startPower;
            }
            else{
                breakPoint = -breakPoint;
            }

            double distanceLong = inchesLong * COUNTS_PER_INCH_REV;
            double distanceShort = inchesShort * COUNTS_PER_INCH_REV;


            double slowdownMarkLong = startingPointLong + (distanceLong - breakPoint);
            double slowdownMarkShort = startingPointShort + (distanceShort - breakPoint);

            double longTarget = startingPointLong + distanceLong;


            double minSpeed = 0.01;

            double speedDropStep = 0.05;

            double originalRight = rightPower;
            double originalLeft = leftPower;


            if (forward) {
                speedIncrementLeft = -speedIncrementLeft;
                speedIncrementRight = - speedIncrementRight;
            }


            double realSpeedLF = startPower;
            double realSpeedLB = startPower;
            double realSpeedRF = startPower;
            double realSpeedRB = startPower;



            boolean accelerating = true;
            boolean slowingDown = false;
            boolean cruising = false;

            boolean stop = false;
            while (!stop && owner.opModeIsActive()) {
                double longReading = leftLong == true ? this.getLeftOdemeter() : this.getRightOdemeter();
                double shortReading = leftLong == true ? this.getRightOdemeter() : this.getLeftOdemeter();

                slowingDown = (forward && (longReading >= slowdownMarkLong || shortReading >= slowdownMarkShort)) ||
                        (forward == false && (longReading <= slowdownMarkLong || shortReading <= slowdownMarkShort));
                if (slowingDown) {
                    //slowing down
                    cruising = false;
                    if (forward) {
                        realSpeedRF = realSpeedRF + speedDropStep;
                        realSpeedRB = realSpeedRB + speedDropStep;
                        realSpeedLF = realSpeedLF + speedDropStep;
                        realSpeedLB = realSpeedLB + speedDropStep;
                        if (realSpeedRF >= -minSpeed || realSpeedRB >= -minSpeed
                                || realSpeedLF >= -minSpeed || realSpeedLB >= -minSpeed) {
                            realSpeedRF = -minSpeed;
                            realSpeedRB = -minSpeed;
                            realSpeedLF = -minSpeed;
                            realSpeedLB = -minSpeed;
                            stop =  longReading >= longTarget;
                            if (stop){
                                break;
                            }
                        }
                    } else {
                        realSpeedRF = realSpeedRF - speedDropStep;
                        realSpeedRB = realSpeedRB - speedDropStep;
                        realSpeedLF = realSpeedLF - speedDropStep;
                        realSpeedLB = realSpeedLB - speedDropStep;
                        if (realSpeedRF <= minSpeed || realSpeedRB <= minSpeed
                                || realSpeedLF <= minSpeed || realSpeedLB <= minSpeed) {
                            realSpeedRF = minSpeed;
                            realSpeedRB = minSpeed;
                            realSpeedLF = minSpeed;
                            realSpeedLB = minSpeed;
                            stop =  longReading <= longTarget;
                            if (stop){
                                break;
                            }
                        }
                    }

                } else if (accelerating) {
                    if ((forward && realSpeedRF + speedIncrementRight >= originalRight && realSpeedLF + speedIncrementLeft >= originalLeft) ||
                            (!forward && realSpeedRF + speedIncrementRight <= originalRight && realSpeedLF + speedIncrementLeft <= originalLeft)){
                        accelerating = true;
                        realSpeedRF += speedIncrementRight;
                        realSpeedRB += speedIncrementRight;
                        realSpeedLF += speedIncrementLeft;
                        realSpeedLB += speedIncrementLeft;

                    }
                    else{
                        if (!cruising){
                            accelerating = false;
                            cruising = true;
                            realSpeedRF = originalRight;
                            realSpeedRB = originalRight;
                            realSpeedLF = originalLeft;
                            realSpeedLB = originalLeft;
                        }
                    }
                }
                if (cruising){
                    //adjust left and right speeds based on the locator
                }

                this.frontLeft.setPower(realSpeedLF * mr.getLF());
                this.backLeft.setPower(realSpeedLB * mr.getLB());
                this.frontRight.setPower(realSpeedRF * mr.getRF());
                this.backRight.setPower(realSpeedRB * mr.getRB());
            }

            this.stop();
        }
    }


    public void moveCurveCalib(BotMoveProfile profile, RobotCoordinatePostiion locator){
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {

            MotorReductionBot mr = profile.getMotorReduction();

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            boolean forward = profile.getDirection() == RobotDirection.Forward;
            boolean leftLong = profile.isLeftLong();


            double startPower = 0;
            double speedIncrementLeft = 0.05;
            double speedIncrementRight = speedIncrementLeft;

            double slowdownMarkLong = profile.getSlowdownMarkLong();
            double slowdownMarkShort = profile.getSlowdownMarkShort();

            double longTarget = profile.getLongTarget();


            double minSpeed = 0.01;

            double lastMileSpeed = 0.2;

            double speedDropStep = 0.05;

            double originalRight = profile.getRealSpeedRight();
            double originalLeft = profile.getRealSpeedLeft();

            if (originalLeft > originalRight) {
                speedIncrementRight = speedIncrementLeft * profile.getSpeedRatio();
            } else if (originalRight > originalLeft) {
                leftLong = false;
                speedIncrementLeft = speedIncrementRight * profile.getSpeedRatio();
            }


            if (forward) {
                speedIncrementLeft = -speedIncrementLeft;
                speedIncrementRight = - speedIncrementRight;
            }


            double realSpeedLF = originalLeft;  //startPower;
            double realSpeedLB = originalLeft;  //startPower;
            double realSpeedRF = originalRight; //startPower;
            double realSpeedRB = originalRight; //startPower;



            boolean accelerating = true;
            boolean slowingDown = false;
            boolean cruising = false;

            boolean stop = false;
            while (!stop && owner.opModeIsActive()) {
                double longReading = leftLong == true ? this.getLeftOdemeter() : this.getRightOdemeter();
                double shortReading = leftLong == true ? this.getRightOdemeter() : this.getLeftOdemeter();

                slowingDown = (forward && (longReading >= slowdownMarkLong || shortReading >= slowdownMarkShort)) ||
                        (forward == false && (longReading <= slowdownMarkLong || shortReading <= slowdownMarkShort));
                if (slowingDown) {
                    //slowing down
                    cruising = false;
                    if (forward) {
                        realSpeedRF = realSpeedRF + speedDropStep;
                        realSpeedRB = realSpeedRB + speedDropStep;
                        realSpeedLF = realSpeedLF + speedDropStep;
                        realSpeedLB = realSpeedLB + speedDropStep;
                        if (realSpeedRF >= -minSpeed || realSpeedRB >= -minSpeed
                                || realSpeedLF >= -minSpeed || realSpeedLB >= -minSpeed) {
                            realSpeedRF = -lastMileSpeed;
                            realSpeedRB = -lastMileSpeed;
                            realSpeedLF = -lastMileSpeed;
                            realSpeedLB = -lastMileSpeed;
                            stop =  longReading >= longTarget;
                            if (stop){
                                break;
                            }
                        }
                    } else {
                        realSpeedRF = realSpeedRF - speedDropStep;
                        realSpeedRB = realSpeedRB - speedDropStep;
                        realSpeedLF = realSpeedLF - speedDropStep;
                        realSpeedLB = realSpeedLB - speedDropStep;
                        if (realSpeedRF <= minSpeed || realSpeedRB <= minSpeed
                                || realSpeedLF <= minSpeed || realSpeedLB <= minSpeed) {
                            realSpeedRF = lastMileSpeed;
                            realSpeedRB = lastMileSpeed;
                            realSpeedLF = lastMileSpeed;
                            realSpeedLB = lastMileSpeed;
                            stop =  longReading <= longTarget;
                            if (stop){
                                break;
                            }
                        }
                    }

                } else if (accelerating) {
                    if ((forward && realSpeedRF + speedIncrementRight >= originalRight && realSpeedLF + speedIncrementLeft >= originalLeft) ||
                            (!forward && realSpeedRF + speedIncrementRight <= originalRight && realSpeedLF + speedIncrementLeft <= originalLeft)){
                        accelerating = true;
                        realSpeedRF += speedIncrementRight;
                        realSpeedRB += speedIncrementRight;
                        realSpeedLF += speedIncrementLeft;
                        realSpeedLB += speedIncrementLeft;

                    }
                    else{
                        if (!cruising){
                            accelerating = false;
                            cruising = true;
                            realSpeedRF = originalRight;
                            realSpeedRB = originalRight;
                            realSpeedLF = originalLeft;
                            realSpeedLB = originalLeft;
                            //start course adjustment
                            locator.setTarget(profile.getTarget());
                        }
                    }
                }
                if (cruising){
                    //adjust left and right speeds based on the locator
                    double  adjustedLeft = locator.getRealSpeedLeft();
                    double  adjustedRight = locator.getRealSpeedRight();
                    if (Math.abs(adjustedLeft) > 0 && Math.abs(adjustedRight) > 0) {
                        realSpeedRF = adjustedRight;
                        realSpeedRB = adjustedRight;
                        realSpeedLF = adjustedLeft;
                        realSpeedLB = adjustedLeft;
                        telemetry.addData("Adj Left", adjustedLeft);
                        telemetry.addData("Adj Right", adjustedRight);
                        leftLong = locator.isLeftLong();
//                        slowdownMarkLong = locator.getSlowdownMarkLong();
//                        slowdownMarkShort = locator.getSlowdownMarkShort();
//                        longTarget = locator.getLongTarget();
                        telemetry.addData("LeftLong", leftLong);
//                        telemetry.addData("SlowdownMarkLong", slowdownMarkLong);
//                        telemetry.addData("SlowdownMarkShort", slowdownMarkShort);
//                        telemetry.addData("longTarget", longTarget);
                    }
                }

                this.frontLeft.setPower(realSpeedLF * mr.getLF());
                this.backLeft.setPower(realSpeedLB * mr.getLB());
                this.frontRight.setPower(realSpeedRF * mr.getRF());
                this.backRight.setPower(realSpeedRB * mr.getRB());
            }

            this.stop();
        }
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
        double horDistance = archDegrees * botConfig.getHorizontalTicksDegree();



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

    public static double getRadius(double angleChange, double chord){
        double alpha = 90 - angleChange;
        double halfChord = chord / 2;
        double cosAlpha = Math.cos(Math.toRadians(alpha));
        double radius = 0;
        if (cosAlpha != 0){
            radius= halfChord / cosAlpha;
        }
        return radius;
    }

    public BotMoveProfile bestRoute(Point start, Point target, RobotDirection direction, double topSpeed, RobotCoordinatePostiion locator){
        double currentX = start.x;
        double currentY = start.y;
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

        double angleChange = Math.abs(targetVector - currentHead);
        if (targetVectorInSquare1 && currentHeadInSquare4 || targetVectorInSquare4 && currentHeadInSquare1){
            angleChange = 360 - angleChange;
        }
        if (angleChange > 90){
            if (direction == RobotDirection.Optimal) {
                currentHead = (currentHead + 180) % 360;
                angleChange = Math.abs(targetVector - currentHead);
                if (targetVectorInSquare1 && currentHeadInSquare4 || targetVectorInSquare4 && currentHeadInSquare1) {
                    angleChange = 360 - angleChange;
                }
                direction = RobotDirection.Backward;
            }
            else {
                //spin
                telemetry.addData("Route",  "Spin");
                return new BotMoveProfile();
            }
        }

        if (direction == RobotDirection.Optimal){
            direction = RobotDirection.Forward;
        }

        if (angleChange == 45){
            //diag
            telemetry.addData("Route",  "Diag");
            return new BotMoveProfile();
        }

        double chord = Math.sqrt(distanceX * distanceX + distanceY * distanceY);

        boolean reduceLeft = false;

        if (targetVector < currentHead) {
            reduceLeft = true;
        }

        if (targetVectorInSquare1 && currentHeadInSquare4 || targetVectorInSquare4 && currentHeadInSquare1 || direction == RobotDirection.Backward){
            reduceLeft = !reduceLeft;
        }

        telemetry.addData("Target Vector", targetVector);
        telemetry.addData("Angle Change", angleChange);



        //check radius.
        double radius = getRadius(angleChange, chord);

        if ((reduceLeft && radius <= this.botConfig.getMinRadiusLeft()) ||
                (reduceLeft == false && radius <= this.botConfig.getMinRadiusRight())) {
            telemetry.addData("Radius", "Too small. Cannot turn. Attempt to strafe");
            // if possible, strafe
            //if not spin
            return new BotMoveProfile();
        }

        BotMoveProfile profile = buildMoveProfile(chord, topSpeed, radius,angleChange,reduceLeft,direction);
        //build request
        BotMoveRequest rq = new BotMoveRequest();
        rq.setTarget(target);
        rq.setTopSpeed(topSpeed);
        rq.setDirection(direction);
        rq.setMotorReduction(profile.getMotorReduction());
        profile.setTarget(rq);
        return profile;
    }

    public BotMoveProfile buildMoveProfile(double chord, double topSpeed, double radius, double angleChange, boolean reduceLeft, RobotDirection direction){
        double longArch = chord;
        double shortArch = chord;
        double speedRatio = 1;
        double lowSpeed = topSpeed;

        if (angleChange > 0) {
            double theta = Math.toRadians(angleChange * 2);
            //double centerArch = theta * radius;
            double wheelDistFromCenter = this.botConfig.getWheelBaseSeparation() / 2;
            longArch = theta * (radius + wheelDistFromCenter);
            shortArch = theta * (radius - wheelDistFromCenter);
            speedRatio = shortArch / longArch;
            lowSpeed = topSpeed * speedRatio;
        }


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
            startingPointLong = this.getLeftOdemeter();
            startingPointShort = this.getRightOdemeter();
        } else if (rightSpeed > leftSpeed) {
            leftLong = false;
            startingPointShort = this.getLeftOdemeter();
            startingPointLong = this.getRightOdemeter();
        }

        double averagePower = (Math.abs(rightSpeed) + Math.abs(leftSpeed))/2;
        averagePower = Math.round(averagePower*10)/10.0;

        MotorReductionBot mr = this.getCalibConfig().getMoveMRForward();
        if (direction == RobotDirection.Backward) {
            mr = this.getCalibConfig().getMoveMRBack();
        }
        double breakPoint = mr.getBreakPoint(averagePower);

        double slowdownMarkLong = startingPointLong + (distanceLong - breakPoint);
        double slowdownMarkShort = startingPointShort + (distanceShort - breakPoint);

        double longTarget = startingPointLong + distanceLong;

        BotMoveProfile profile = new BotMoveProfile();
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


    public void moveToCoordinate(Point start, Point target, RobotDirection direction, double topSpeed, RobotCoordinatePostiion locator){
            //X and Y are front center of the robot
        double currentX = start.x;
        double currentY = start.y;
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

        double chord = Math.sqrt(distanceX * distanceX + distanceY * distanceY);

        telemetry.addData("Chord", chord);

        telemetry.addData("Target Vector", targetVector);

        double angleChange = Math.abs(targetVector - currentHead);
        if (targetVectorInSquare1 && currentHeadInSquare4 || targetVectorInSquare4 && currentHeadInSquare1){
            angleChange = 360 - angleChange;
        }
//            double sign = -currentHead / Math.abs(currentHead);
//            if (angleChange > 90) {
//                if (direction == RobotDirection.Backward) {
////                    currentHead = (180 - Math.abs(currentHead)) * sign;
//                    currentHead = (360 + currentHead) % 360;
//                } else {
//                    //spin
////                    spinH(targetVector, topSpeed);
////                    //todo: make more precise
////                    currentHead = targetVector;
//                }
//                angleChange = Math.abs(targetVector - currentHead);
//            }

        telemetry.addData("CurrentHead", currentHead);
        telemetry.addData("AngleChange", angleChange);

        boolean reduceLeft = false;

        if (targetVector < currentHead) {
            reduceLeft = true;
        }

        if (targetVectorInSquare1 && currentHeadInSquare4 || targetVectorInSquare4 && currentHeadInSquare1 || direction == RobotDirection.Backward){
            reduceLeft = !reduceLeft;
        }

        telemetry.addData("reduceLeft", reduceLeft);

        double alpha = 90 - angleChange;
        double theta = Math.toRadians(angleChange * 2);
        double halfChord = chord / 2;
        double cosAlpha = Math.cos(Math.toRadians(alpha));
        double radius = 0;
        if (cosAlpha != 0){
            radius= halfChord / cosAlpha;
        }


        if ((reduceLeft && radius <= this.botConfig.getMinRadiusLeft()) ||
                (reduceLeft == false && radius <= this.botConfig.getMinRadiusRight())) {
            telemetry.addData("Radius", "Too small. Cannot turn");
        } else {
            double longArch = chord;
            double shortArch = chord;
            double speedRatio = 1;
            double lowSpeed = topSpeed;

            if (angleChange > 0) {
                //double centerArch = theta * radius;
                double wheelDistFromCenter = this.botConfig.getWheelBaseSeparation() / 2;
                longArch = theta * (radius + wheelDistFromCenter);
                shortArch = theta * (radius - wheelDistFromCenter);
                speedRatio = shortArch / longArch;
                lowSpeed = topSpeed * speedRatio;
            }

            if (direction == RobotDirection.Backward){
                shortArch = -shortArch;
                longArch = -longArch;
            }

            telemetry.addData("Radius", radius);
            telemetry.addData("Theta", theta);
            telemetry.addData("longArch", longArch);
            telemetry.addData("shortArch", shortArch);
            double leftSpeed, rightSpeed;
            if (reduceLeft) {
                telemetry.addData("right speed", topSpeed);
                telemetry.addData("left speed", lowSpeed);
                leftSpeed = lowSpeed;
                rightSpeed = topSpeed;
            } else {
                telemetry.addData("left speed", topSpeed);
                telemetry.addData("right speed", lowSpeed);
                leftSpeed = topSpeed;
                rightSpeed = lowSpeed;
            }


            /////////////////////
            moveCurveCalib(leftSpeed, rightSpeed, speedRatio, shortArch, longArch, direction, locator);
            telemetry.addData("Start X", start.x);
            telemetry.addData("Start Y", start.y);
            telemetry.addData("Target X", target.x);
            telemetry.addData("Target Y", target.y);
            telemetry.addData("Distance", Geometry.getDistance(locator.getXInches(), locator.getYInches(), target.x, target.y));
            telemetry.addData("Actual X", locator.getXInches());
            telemetry.addData("Actual Y", locator.getYInches());
            telemetry.addData("front center X", locator.getFrontCenterXInches());
            telemetry.addData("front center Y", locator.getFrontCenterYInches());
            telemetry.addData("Head", locator.getOrientation());
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

    public double strafeTo(double speed, double inches, boolean left) {
        double currentPos = this.getHorizontalOdemeter();
        double distance = inches * COUNTS_PER_INCH_REV;

        MotorReductionBot calib = null;
        if (left) {
            calib = getCalibConfig().getStrafeLeftReduction();
        }
        else{
            calib = getCalibConfig().getStrafeRightReduction();
        }



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
                this.backLeft.setPower(-speed*calib.getLB());
                this.backRight.setPower(speed*calib.getRB());
                this.frontLeft.setPower(speed*calib.getLF());
                this.frontRight.setPower(-speed*calib.getRF());
            }
            else{
                this.backLeft.setPower(speed*calib.getLB());
                this.backRight.setPower(-speed*calib.getRB());
                this.frontLeft.setPower(-speed*calib.getLF());
                this.frontRight.setPower(speed*calib.getRF());
            }
        }

        stop();
        double newPos = this.getHorizontalOdemeter();
        double diff = Math.abs(newPos - target);
        overage = diff/distance*100;
        return overage;
    }

    public double strafeToCalib(double speed, double inches, boolean left, MotorReductionBot calib) {
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
                this.backLeft.setPower(-speed*calib.getLB());
                this.backRight.setPower(speed*calib.getRB());
                this.frontLeft.setPower(speed*calib.getLF());
                this.frontRight.setPower(-speed*calib.getRF());
            }
            else{
                this.backLeft.setPower(speed*calib.getLB());
                this.backRight.setPower(-speed*calib.getRB());
                this.frontLeft.setPower(-speed*calib.getLF());
                this.frontRight.setPower(speed*calib.getRF());
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

    public void diagToCalib(double speed, double lowSpeed, double diagInches, boolean leftAxis, MotorReductionBot calib){
        if (backLeft != null && backRight!= null && frontLeft != null && frontRight != null) {


            double leftOdoStart = getLeftOdemeter();
            double rightOdoStart = getRightOdemeter();
            double horOdoStart = getHorizontalOdemeter();

            double distance = Math.abs(diagInches * COUNTS_PER_INCH_REV);

//            double horDistance = distance * Math.sin(Math.toRadians(targetAngle));
//            double verDistance = distance * Math.cos(Math.toRadians(targetAngle));
//
//
//
//            boolean angleChange = lowSpeed > 0;

            double power = speed;

            if (diagInches > 0){
                power = -power;
                lowSpeed = -lowSpeed;
            }

            boolean stop = false;


            while(!stop && owner.opModeIsActive()){
                double leftOdo = getLeftOdemeter();
                double rightOdo = getRightOdemeter();
                double horOdo = getHorizontalOdemeter();
                double leftDistActual = Math.abs(leftOdo - leftOdoStart);
                double rightDistActual = Math.abs(rightOdo - rightOdoStart);
                double horDistActual = Math.abs(horOdo - horOdoStart);

                double catet = leftDistActual;
                if (rightDistActual > catet){
                    catet = rightDistActual;
                }

                double hyp = Math.sqrt(catet* catet + horDistActual * horDistActual );

                if (hyp >= distance) {
                    break;
                }

//                if (angleChange) {
//                    if (hyp >= distance) {
//                        break;
//                    }
//                }else {
//                    if (leftDistActual >= verDistance || rightDistActual >= verDistance || horDistActual >= horDistance) {
//                        break;
//                    }
//                }

                this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


                if (!leftAxis) {
                    this.frontLeft.setPower(power*calib.getLF());
                    this.backRight.setPower(power*calib.getRB());

                    this.backLeft.setPower(lowSpeed*calib.getLB());
                    this.frontRight.setPower(lowSpeed*calib.getRF());
                }
                else{
                    this.backLeft.setPower(power*calib.getLB());
                    this.frontRight.setPower(power*calib.getRF());

                    this.frontLeft.setPower(lowSpeed*calib.getLF());
                    this.backRight.setPower(lowSpeed*calib.getRB());
                }
            }
        }
        this.stop();
    }

    public void diagTo(double speed,  double diagInches, double angle,  DiagCalibConfig calibCofig){
        if (backLeft != null && backRight!= null && frontLeft != null && frontRight != null) {



            double leftOdoStart = getLeftOdemeter();
            double rightOdoStart = getRightOdemeter();
            double horOdoStart = getHorizontalOdemeter();

            double distance = Math.abs(diagInches * COUNTS_PER_INCH_REV);

//            double horDistance = distance * Math.sin(Math.toRadians(targetAngle));
//            double verDistance = distance * Math.cos(Math.toRadians(targetAngle));
//

            boolean leftAxis = angle > 0;
            double lowSpeed = calibCofig.getSpeedPerDegree() * angle;

            double power = speed;

            if (diagInches > 0){
                power = -power;
                lowSpeed = -lowSpeed;
            }

            boolean stop = false;


            while(!stop && owner.opModeIsActive()){
                double leftOdo = getLeftOdemeter();
                double rightOdo = getRightOdemeter();
                double horOdo = getHorizontalOdemeter();
                double leftDistActual = Math.abs(leftOdo - leftOdoStart);
                double rightDistActual = Math.abs(rightOdo - rightOdoStart);
                double horDistActual = Math.abs(horOdo - horOdoStart);

                double catet = leftDistActual;
                if (rightDistActual > catet){
                    catet = rightDistActual;
                }

                double hyp = Math.sqrt(catet* catet + horDistActual * horDistActual );

                if (hyp >= distance) {
                    break;
                }

//                if (angleChange) {
//                    if (hyp >= distance) {
//                        break;
//                    }
//                }else {
//                    if (leftDistActual >= verDistance || rightDistActual >= verDistance || horDistActual >= horDistance) {
//                        break;
//                    }
//                }

                this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


                if (!leftAxis) {
                    this.frontLeft.setPower(power);
                    this.backRight.setPower(power);

                    this.backLeft.setPower(lowSpeed);
                    this.frontRight.setPower(lowSpeed);
                }
                else{
                    this.backLeft.setPower(power);
                    this.frontRight.setPower(power);

                    this.frontLeft.setPower(lowSpeed);
                    this.backRight.setPower(lowSpeed);
                }
            }
        }
        this.stop();
    }



    public void initCalibData() throws Exception {
        File calibFile = AppUtil.getInstance().getSettingsFile(BotCalibConfig.BOT_CALIB_CONFIG);
        if (calibFile.exists()){
            String data = ReadWriteFile.readFile(calibFile);
            botConfig = BotCalibConfig.deserialize(data);
            if (botConfig == null){
                throw new Exception("Calibration data does not exist. Run calibration first");
            }
            telemetry.addData("Bot Config", "Initialized");
            telemetry.update();
        }
        else{
            throw new Exception("Calibration data does not exist. Run calibration first");
        }
    }

    public BotCalibConfig getCalibConfig(){
        if (botConfig == null) {
            File calibFile = AppUtil.getInstance().getSettingsFile(BotCalibConfig.BOT_CALIB_CONFIG);
            if (calibFile.exists()) {
                String data = ReadWriteFile.readFile(calibFile);
                botConfig = BotCalibConfig.deserialize(data);
            }
        }
        return botConfig;
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
            led.init(hwMap, telemetry);
        }
        return led;
    }


}
