package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.gamefield.GameStats;
import org.firstinspires.ftc.teamcode.skills.DetectionInterface;
import org.firstinspires.ftc.teamcode.skills.Gyro;

/**
 * Created by sjeltuhin on 9/12/17.
 */

public class TieBot {
    Telemetry telemetry;
    public DcMotor leftDriveBack = null;
    public DcMotor rightDriveBack = null;

    public DcMotor leftDriveFront = null;
    public DcMotor rightDriveFront = null;

    public DcMotor towerDrive = null;

    public DcMotor craneDrive = null;

    public DcMotor intakeLeft = null;
    public DcMotor intakeRight = null;


    public Servo capstone = null;

    public Servo lockStone = null;

    public Servo rotateStone = null;

    public Servo hookLeft = null;

    public Servo hookRight = null;

    private Gyro gyro = null;

    private DistanceSensor rangeBack;
    private DistanceSensor rangeLeft;
    private DistanceSensor rangeRight;
    private DistanceSensor stoneSensor;


    private ElapsedTime     runtime = new ElapsedTime();



    public double ANTI_GRAVITY_POWER = -0.5;

    public double DRIVE_SPEED = 0.99;

    static final double     STRAFE_COUNT_INCH    =  20;    // Rev Core Hex motor
    public static final double     PIVOT_SPEED = 0.2;

    public static final double     INTAKE_PIVOT_SPEED = 1;
    public static final double     INTAKE_PIVOT_SPEED_UP = 1;

    private static final double MAX_RANGE = 200;

    public static final double ROBOT_LENGTH = 18;
    public static final double ROBOT_WIDTH = 18;



    //REV
    static final double calibration = 1.07;

    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // Rev Core Hex motor
    static final double     DRIVE_GEAR_REDUCTION    = 0.517 ;     // This is < 1.0 if geared UP. was 2 in the sample
    static final double     WHEEL_DIAMETER_INCHES   = 4.05 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH_REV     = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI) * calibration;  //22.64

    private int cranePosition = 0;
    static final int MAX_CRANE_POS = 1200;
    static final double     COUNTS_PER_MOTOR_CRANE    = 117 ;

    double capPos = -1;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public TieBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry telemetry) throws Exception{
        // Save reference to Hardware map
        hwMap = ahwMap;
        this.telemetry = telemetry;
        try {
            // Define and Initialize Motors
            leftDriveBack = hwMap.get(DcMotor.class, "left_drive_back");
            rightDriveBack = hwMap.get(DcMotor.class, "right_drive_back");
            leftDriveFront = hwMap.get(DcMotor.class, "left_drive_front");
            rightDriveFront = hwMap.get(DcMotor.class, "right_drive_front");
        }
        catch (Exception ex){
            //issues accessing drive resources
//            throw new Exception("Issues accessing drive resources. Check the controller config", ex);
        }

        try{
            towerDrive = hwMap.get(DcMotor.class, "tower");
        }
        catch (Exception ex){
            throw new Exception("Issues accessing tower. Check the controller config", ex);
        }
        telemetry.addData("Init", "towerDrive");


        try{
            craneDrive = hwMap.get(DcMotor.class, "crane");
        }
        catch (Exception ex){
            throw new Exception("Issues accessing platform. Check the controller config", ex);
        }
        telemetry.addData("Init", "craneDrive");

        try{
            intakeLeft = hwMap.get(DcMotor.class, "intake-left");
            if (intakeLeft != null){
                intakeLeft.setDirection(DcMotor.Direction.REVERSE);
                intakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                intakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                intakeLeft.setPower(0);
            }
        }
        catch (Exception ex){
            throw new Exception("Issues accessing left intake. Check the controller config", ex);
        }


        try{
            intakeRight = hwMap.get(DcMotor.class, "intake-right");
            if (intakeRight != null){
                intakeRight.setDirection(DcMotor.Direction.FORWARD);
                intakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                intakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                intakeRight.setPower(0);
            }
        }
        catch (Exception ex){
            throw new Exception("Issues accessing right intake. Check the controller config", ex);
        }


        if (leftDriveBack != null) {
            leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        }

        if (rightDriveBack != null) {
            rightDriveBack.setDirection(DcMotor.Direction.FORWARD);
        }

        if (leftDriveFront != null) {
            leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        }

        if (rightDriveFront != null) {
            rightDriveFront.setDirection(DcMotor.Direction.FORWARD);
        }

        resetEncoders();

        if (leftDriveBack != null && rightDriveBack!= null && leftDriveFront != null && rightDriveFront != null) {

            leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Set all motors to zero power
            leftDriveBack.setPower(0);
            rightDriveBack.setPower(0);
            leftDriveFront.setPower(0);
            rightDriveFront.setPower(0);
        }

        if (towerDrive != null){
            towerDrive.setDirection(DcMotor.Direction.REVERSE);
            towerDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            towerDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            towerDrive.setPower(0);
        }


        if (craneDrive != null){
            craneDrive.setDirection(DcMotor.Direction.REVERSE);
            craneDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            craneDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            craneDrive.setPower(0);
        }



        try{
            capstone = hwMap.get(Servo.class, "caps");
            if (capstone != null){
                capPos = this.capstone.getPosition();
            }
        }
        catch (Exception ex){
            throw new Exception("Issues accessing capstone servo. Check the controller config", ex);
        }

        try{
            lockStone = hwMap.get(Servo.class, "lock_servo");
//            if (lockStone != null){
//                lockStone.setPosition(1);
//            }
        }
        catch (Exception ex){
            throw new Exception("Issues accessing lock servo. Check the controller config", ex);
        }

        try{
            rotateStone = hwMap.get(Servo.class, "rotate_servo");
//            if (lockStone != null){
//                lockStone.setPosition(1);
//            }
        }
        catch (Exception ex){
            throw new Exception("Issues accessing rotate servo. Check the controller config", ex);
        }


        try{
            hookLeft = hwMap.get(Servo.class, "hook_left");
        }
        catch (Exception ex){
            throw new Exception("Issues accessing hook left servo. Check the controller config", ex);
        }

        try{
            hookRight = hwMap.get(Servo.class, "hook_right");
        }
        catch (Exception ex){
            throw new Exception("Issues accessing hook right servo. Check the controller config", ex);
        }
    }

    public void startGyro(HardwareMap ahwMap, Telemetry telemetry){
        setGyro(new Gyro());
        getGyro().init(this, hwMap, telemetry);
    }

    public void initSensors() throws Exception {
        try{
            rangeBack = hwMap.get(DistanceSensor.class, "range_back");
        }
        catch (Exception ex){
            throw new Exception("Issues accessing range sensor back . Check the controller config", ex);
        }

        try{
            rangeLeft = hwMap.get(DistanceSensor.class, "range_left");
        }
        catch (Exception ex){
            throw new Exception("Issues accessing range sensor left. Check the controller config", ex);
        }

        try{
            rangeRight = hwMap.get(DistanceSensor.class, "range_right");
        }
        catch (Exception ex){
            throw new Exception("Issues accessing range sensor right. Check the controller config", ex);
        }

        initStoneSensor();
    }

    public void initStoneSensor() throws Exception {
        try{
            stoneSensor = hwMap.get(DistanceSensor.class, "stone_sensor");
        }
        catch (Exception ex){
            throw new Exception("Issues accessing range sensor right. Check the controller config", ex);
        }
    }

    protected void resetEncoders(){
        if (leftDriveBack != null && rightDriveBack!= null && leftDriveFront != null && rightDriveFront != null)
        {
            leftDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void initMode(DcMotor.RunMode mode){
        if (leftDriveBack != null && rightDriveBack!= null && leftDriveFront != null && rightDriveFront != null) {
            leftDriveBack.setMode(mode);
            rightDriveBack.setMode(mode);
            leftDriveFront.setMode(mode);
            rightDriveFront.setMode(mode);
        }
    }

    public void stop (){
        if (leftDriveBack != null && rightDriveBack!= null && leftDriveFront != null && rightDriveFront != null) {
            this.leftDriveBack.setPower(0);
            this.rightDriveBack.setPower(0);
            this.leftDriveFront.setPower(0);
            this.rightDriveFront.setPower(0);
        }
    }

    public void move(double drive, double turn){
        if (leftDriveBack != null && rightDriveBack!= null && leftDriveFront != null && rightDriveFront != null) {
            double rightPower = Range.clip(drive + (turn * 0.85), -1.0, 1.0);
            double leftPower = Range.clip(drive - (turn * 0.85), -1.0, 1.0);
            if ((drive > 0 && drive <= 4 )|| (turn > 0 && turn <= 4)){
                rightPower = rightPower * rightPower * rightPower;
                leftPower = leftPower * leftPower * leftPower;
            }
            //use cubic modifier
//        rightPower = rightPower*rightPower*rightPower;
//        leftPower = leftPower*leftPower*leftPower;

            this.leftDriveBack.setPower(leftPower);
            this.rightDriveBack.setPower(rightPower);
            this.leftDriveFront.setPower(leftPower);
            this.rightDriveFront.setPower(rightPower);
            telemetry.addData("Motors", "Left: %.0f", leftPower);
            telemetry.addData("Motors", "Right: %.0f", rightPower);
            telemetry.addData("Motors", "Turn: %.0f", turn);
            telemetry.addData("Motors", "LeftFront from %7d", leftDriveFront.getCurrentPosition());
            telemetry.addData("Motors", "LeftBack from %7d", leftDriveBack.getCurrentPosition());
            telemetry.addData("Motors", "RightFront from %7d", rightDriveFront.getCurrentPosition());
            telemetry.addData("Motors", "RightBack from %7d", rightDriveBack.getCurrentPosition());
        }
    }


    public void strafeLeft(double speed){
        if (leftDriveBack != null && rightDriveBack!= null && leftDriveFront != null && rightDriveFront != null) {
            double power = Range.clip(speed, -1.0, 1.0);
            power = power * power * power;
            this.leftDriveBack.setPower(power);
            this.rightDriveBack.setPower(-power);
            this.leftDriveFront.setPower(-power);
            this.rightDriveFront.setPower(power);
            telemetry.addData("Motors", "Front: %.0f", power);
            telemetry.addData("Motors", "Back: %.0f", power);
        }
    }

    public void strafeRight(double speed){
        if (leftDriveBack != null && rightDriveBack!= null && leftDriveFront != null && rightDriveFront != null) {
            double power = Range.clip(speed, -1.0, 1.0);
            power = power * power * power;
            this.leftDriveBack.setPower(-power);
            this.rightDriveBack.setPower(power);
            this.leftDriveFront.setPower(power);
            this.rightDriveFront.setPower(-power);
            telemetry.addData("Motors", "Front: %.0f", power);
            telemetry.addData("Motors", "Back: %.0f", power);
        }
    }

    public void diagLeft(double speed){
        if (leftDriveBack != null && rightDriveBack!= null && leftDriveFront != null && rightDriveFront != null) {
            double power = Range.clip(speed, -1.0, 1.0);
            this.leftDriveFront.setPower(power);
            this.rightDriveBack.setPower(power);

            this.leftDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.rightDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            this.leftDriveBack.setPower(0);
            this.rightDriveFront.setPower(0);
        }
    }

    public void diagRight(double speed){
        if (leftDriveBack != null && rightDriveBack!= null && leftDriveFront != null && rightDriveFront != null) {
            double power = Range.clip(speed, -1.0, 1.0);

            this.leftDriveBack.setPower(power);
            this.rightDriveFront.setPower(power);

            this.leftDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.rightDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            this.leftDriveFront.setPower(0);
            this.rightDriveBack.setPower(0);
        }
    }

    public void pivotLeft(double speed){
        if (leftDriveBack != null && rightDriveBack!= null && leftDriveFront != null && rightDriveFront != null) {
            double power = Range.clip(speed, -1.0, 1.0);

            this.leftDriveBack.setPower(-power);
            this.rightDriveBack.setPower(power);
            this.leftDriveFront.setPower(-power);
            this.rightDriveFront.setPower(power);
            telemetry.addData("Motors", "Left: %7d Right: %7d", leftDriveFront.getCurrentPosition(), rightDriveFront.getCurrentPosition());
            telemetry.update();
        }
    }

    public void pivotRight(double speed){
        if (leftDriveBack != null && rightDriveBack!= null && leftDriveFront != null && rightDriveFront != null) {
            double power = Range.clip(speed, -1.0, 1.0);

            this.leftDriveBack.setPower(power);
            this.rightDriveBack.setPower(-power);
            this.leftDriveFront.setPower(power);
            this.rightDriveFront.setPower(-power);
            telemetry.addData("Motors", "Lef: %7d Right: %7d", leftDriveFront.getCurrentPosition(), rightDriveFront.getCurrentPosition());
            telemetry.update();
        }
    }

    public void turnLeft(double speed){
        if (leftDriveBack != null && rightDriveBack!= null && leftDriveFront != null && rightDriveFront != null) {
            this.leftDriveBack.setPower(0);
            this.rightDriveBack.setPower(speed);
            this.leftDriveFront.setPower(0);
            this.rightDriveFront.setPower(speed);
        }
    }

    public void turnRight(double speed){
        if (leftDriveBack != null && rightDriveBack!= null && leftDriveFront != null && rightDriveFront != null) {
            this.leftDriveBack.setPower(speed);
            this.rightDriveBack.setPower(0);
            this.leftDriveFront.setPower(speed);
            this.rightDriveFront.setPower(0);
        }
    }

    public boolean motorsBusy(){
        return this.leftDriveBack.isBusy() && this.rightDriveBack.isBusy()
                && this.leftDriveFront.isBusy() && this.rightDriveFront.isBusy();

    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutMS, LinearOpMode caller) {

        try {
            // Determine new target position, and pass to motor controller
            int newLeftTarget = this.leftDriveBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH_REV);
            int newRightTarget = this.rightDriveBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH_REV);
            int newLeftFrontTarget = this.leftDriveFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH_REV);
            int newRightFrontTarget = this.rightDriveFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH_REV);

            this.leftDriveBack.setTargetPosition(newLeftTarget);
            this.rightDriveBack.setTargetPosition(newRightTarget);
            this.leftDriveFront.setTargetPosition(newLeftFrontTarget);
            this.rightDriveFront.setTargetPosition(newRightFrontTarget);


            // Turn On RUN_TO_POSITION
            leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            ((DcMotorEx)(this.leftDriveBack)).setTargetPositionTolerance((int)COUNTS_PER_INCH_REV);
            ((DcMotorEx)(this.rightDriveBack)).setTargetPositionTolerance((int)COUNTS_PER_INCH_REV);
            ((DcMotorEx)(this.leftDriveFront)).setTargetPositionTolerance((int)COUNTS_PER_INCH_REV);
            ((DcMotorEx)(this.rightDriveFront)).setTargetPositionTolerance((int)COUNTS_PER_INCH_REV);

            // reset the timeout time and start motion.
            runtime.reset();
            this.leftDriveBack.setPower(Math.abs(speed));
            this.rightDriveBack.setPower(Math.abs(speed));
            this.leftDriveFront.setPower(Math.abs(speed));
            this.rightDriveFront.setPower(Math.abs(speed));

            boolean stop = false;
            boolean leftMove = leftInches != 0;
            boolean rightMove = rightInches != 0;
            while (!stop) {
                boolean timeUp = timeoutMS > 0 && runtime.milliseconds() >= timeoutMS;
                stop = !caller.opModeIsActive() || timeUp || ((leftMove && !this.leftDriveBack.isBusy()) || (rightMove && !this.rightDriveBack.isBusy())
                || (leftMove && !this.leftDriveFront.isBusy()) || (rightMove && !this.rightDriveFront.isBusy()));

            }

            this.stop();


            // Turn off RUN_TO_POSITION
            leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception ex){
            telemetry.addData("Issues running with encoders to position", ex);
            telemetry.update();
        }
    }

    public void encoderStartMove(double speed,
                             double leftInches, double rightInches,
                             double timeoutMS, LinearOpMode caller) {

        try {
            // Determine new target position, and pass to motor controller
            int newLeftTarget = this.leftDriveBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH_REV);
            int newRightTarget = this.rightDriveBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH_REV);
            int newLeftFrontTarget = this.leftDriveFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH_REV);
            int newRightFrontTarget = this.rightDriveFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH_REV);

            this.leftDriveBack.setTargetPosition(newLeftTarget);
            this.rightDriveBack.setTargetPosition(newRightTarget);
            this.leftDriveFront.setTargetPosition(newLeftFrontTarget);
            this.rightDriveFront.setTargetPosition(newRightFrontTarget);


            // Turn On RUN_TO_POSITION
            leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            ((DcMotorEx)(this.leftDriveBack)).setTargetPositionTolerance((int)COUNTS_PER_INCH_REV);
            ((DcMotorEx)(this.rightDriveBack)).setTargetPositionTolerance((int)COUNTS_PER_INCH_REV);
            ((DcMotorEx)(this.leftDriveFront)).setTargetPositionTolerance((int)COUNTS_PER_INCH_REV);
            ((DcMotorEx)(this.rightDriveFront)).setTargetPositionTolerance((int)COUNTS_PER_INCH_REV);

            // reset the timeout time and start motion.
            runtime.reset();
            this.leftDriveBack.setPower(Math.abs(speed));
            this.rightDriveBack.setPower(Math.abs(speed));
            this.leftDriveFront.setPower(Math.abs(speed));
            this.rightDriveFront.setPower(Math.abs(speed));
        }
        catch (Exception ex){
            telemetry.addData("Issues running with encoders to position", ex);
            telemetry.update();
        }
    }

    public void encoderStopMove(){

        this.stop();


        // Turn off RUN_TO_POSITION
        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderDriveGyro(double speed,
                             double leftInches, double rightInches,
                             double timeoutS, LinearOpMode caller) {

        try {
            // Determine new target position, and pass to motor controller
            int newLeftTarget = this.leftDriveBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH_REV);
            int newRightTarget = this.rightDriveBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH_REV);
            int newLeftFrontTarget = this.leftDriveFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH_REV);
            int newRightFrontTarget = this.rightDriveFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH_REV);

            this.leftDriveBack.setTargetPosition(newLeftTarget);
            this.rightDriveBack.setTargetPosition(newRightTarget);
            this.leftDriveFront.setTargetPosition(newLeftFrontTarget);
            this.rightDriveFront.setTargetPosition(newRightFrontTarget);


            // Turn On RUN_TO_POSITION
            leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            ((DcMotorEx)(this.leftDriveBack)).setTargetPositionTolerance((int)COUNTS_PER_INCH_REV);
            ((DcMotorEx)(this.rightDriveBack)).setTargetPositionTolerance((int)COUNTS_PER_INCH_REV);
            ((DcMotorEx)(this.leftDriveFront)).setTargetPositionTolerance((int)COUNTS_PER_INCH_REV);
            ((DcMotorEx)(this.rightDriveFront)).setTargetPositionTolerance((int)COUNTS_PER_INCH_REV);

            // reset the timeout time and start motion.
            runtime.reset();
            this.leftDriveBack.setPower(Math.abs(speed));
            this.rightDriveBack.setPower(Math.abs(speed));
            this.leftDriveFront.setPower(Math.abs(speed));
            this.rightDriveFront.setPower(Math.abs(speed));

            boolean stop = false;
            boolean leftMove = leftInches != 0;
            boolean rightMove = rightInches != 0;
            boolean resume = false;
            while (!stop) {
                boolean timeUp = timeoutS > 0 && runtime.seconds() >= timeoutS;
                stop = !caller.opModeIsActive() || timeUp || ((leftMove && !this.leftDriveBack.isBusy()) || (rightMove && !this.rightDriveBack.isBusy())
                        || (leftMove && !this.leftDriveFront.isBusy()) || (rightMove && !this.rightDriveFront.isBusy()));

                if(getGyro().isOffCourse()){
                    leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    getGyro().fixHeading(0.3, caller);
                    resume = true;
                }else if (resume){
                    this.leftDriveBack.setTargetPosition(newLeftTarget);
                    this.rightDriveBack.setTargetPosition(newRightTarget);
                    this.leftDriveFront.setTargetPosition(newLeftFrontTarget);
                    this.rightDriveFront.setTargetPosition(newRightFrontTarget);


                    // Turn On RUN_TO_POSITION
                    leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    this.leftDriveBack.setPower(Math.abs(speed));
                    this.rightDriveBack.setPower(Math.abs(speed));
                    this.leftDriveFront.setPower(Math.abs(speed));
                    this.rightDriveFront.setPower(Math.abs(speed));
                    resume = false;
                }
            }

            telemetry.addData("Motors", "Going to stop");
            telemetry.update();
            this.stop();
            telemetry.addData("Motors", "Stopped");
            telemetry.update();

            // Turn off RUN_TO_POSITION
            leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception ex){
            telemetry.addData("Issues running with encoders to position", ex);
            telemetry.update();
        }
    }


    public void encoderTurn(double turn,
                             double leftInches, double rightInches,
                             double timeoutS) {

        try {
            // Determine new target position, and pass to motor controller
            int newLeftTarget = 0;
            int newRightTarget = 0;
            int newLeftFrontTarget = 0;
            int newRightFrontTarget = 0;
            if (turn <=0 ) {
                newLeftTarget = this.leftDriveBack.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH_REV);
                newRightTarget = this.rightDriveBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH_REV);
                newLeftFrontTarget = this.leftDriveFront.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH_REV);
                newRightFrontTarget = this.rightDriveFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH_REV);
            }
            else{
                newLeftTarget = this.leftDriveBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH_REV);
                newRightTarget = this.rightDriveBack.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH_REV);
                newLeftFrontTarget = this.leftDriveFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH_REV);
                newRightFrontTarget = this.rightDriveFront.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH_REV);
            }

            this.leftDriveBack.setTargetPosition(newLeftTarget);
            this.rightDriveBack.setTargetPosition(newRightTarget);
            this.leftDriveFront.setTargetPosition(newLeftFrontTarget);
            this.rightDriveFront.setTargetPosition(newRightFrontTarget);


            // Turn On RUN_TO_POSITION
            leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            this.leftDriveBack.setPower(Math.abs(turn));
            this.rightDriveBack.setPower(Math.abs(turn));
            this.leftDriveFront.setPower(Math.abs(turn));
            this.rightDriveFront.setPower(Math.abs(turn));

            boolean stop = false;
            boolean leftMove = leftInches != 0;
            boolean rightMove = rightInches != 0;
            while (!stop) {
                boolean timeUp = timeoutS > 0 && runtime.seconds() >= timeoutS;
                stop = timeUp || ((leftMove && !this.leftDriveBack.isBusy()) || (rightMove && !this.rightDriveBack.isBusy())
                        || (leftMove && !this.leftDriveFront.isBusy()) || (rightMove && !this.rightDriveFront.isBusy()));

                telemetry.addData("Motors", "Starting encoder drive. Left: %.2f, Right:%.2f", leftInches, rightInches);
                telemetry.addData("Motors", "LeftFront from %7d to %7d", leftDriveFront.getCurrentPosition(), newLeftFrontTarget);
                telemetry.addData("Motors", "LeftBack from %7d to %7d", leftDriveBack.getCurrentPosition(), newLeftTarget);
                telemetry.addData("Motors", "RightFront from %7d to %7d", rightDriveFront.getCurrentPosition(), newRightFrontTarget);
                telemetry.addData("Motors", "RightBack from %7d to %7d", rightDriveBack.getCurrentPosition(), newRightTarget);
                telemetry.update();
            }

            telemetry.addData("Motors", "Going to stop");
            telemetry.update();
            this.stop();
            telemetry.addData("Motors", "Stopped");
            telemetry.update();

            // Turn off RUN_TO_POSITION
            leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception ex){
            telemetry.addData("Issues running with encoders to position", ex);
            telemetry.update();
        }
    }

    public int getStrafeIncrement(double moveTo){
        int increment = (int) (moveTo * STRAFE_COUNT_INCH);
        return increment;
    }

    public int getDriveIncrement(double moveTo){
       return  (int) (moveTo * this.COUNTS_PER_INCH_REV);
    }

    public double getPositionDiffInches(int start, int finish){
        return  Math.abs(finish - start)/COUNTS_PER_INCH_REV;
    }
    public void encoderStrafe(double speed,
                             double distanceInches,
                             double timeoutS) {

        try {
            double val = Math.abs(distanceInches);
            int newLeftTarget = 0 ;
            int newRightTarget = 0;
            int newLeftFrontTarget = 0;
            int newRightFrontTarget = 0;

            int increment = (int) (val * STRAFE_COUNT_INCH);
            if (distanceInches < 0){
                //going left
                newLeftTarget = this.leftDriveBack.getCurrentPosition() + increment;
                newRightTarget = this.rightDriveBack.getCurrentPosition() - increment;
                newLeftFrontTarget = this.leftDriveFront.getCurrentPosition() - increment;
                newRightFrontTarget = this.rightDriveFront.getCurrentPosition() + increment;
            }
            else{
                //going right
                newLeftTarget = this.leftDriveBack.getCurrentPosition() - increment;
                newRightTarget = this.rightDriveBack.getCurrentPosition() + increment;
                newLeftFrontTarget = this.leftDriveFront.getCurrentPosition() + increment;
                newRightFrontTarget = this.rightDriveFront.getCurrentPosition() - increment;
            }


            this.leftDriveBack.setTargetPosition(newLeftTarget);
            this.rightDriveBack.setTargetPosition(newRightTarget);
            this.leftDriveFront.setTargetPosition(newLeftFrontTarget);
            this.rightDriveFront.setTargetPosition(newRightFrontTarget);


            // Turn On RUN_TO_POSITION
            leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            this.leftDriveBack.setPower(Math.abs(speed));
            this.rightDriveBack.setPower(Math.abs(speed));
            this.leftDriveFront.setPower(Math.abs(speed));
            this.rightDriveFront.setPower(Math.abs(speed));

            boolean stop = false;
            while (!stop) {
                boolean timeUp = timeoutS > 0 && runtime.seconds() >= timeoutS;
                stop = timeUp || (!this.leftDriveBack.isBusy() || !this.rightDriveBack.isBusy()
                        || !this.leftDriveFront.isBusy() || !this.rightDriveFront.isBusy());
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Back: %7d :%7d front: %7d :%7d",
                        this.leftDriveBack.getCurrentPosition(),
                        this.rightDriveBack.getCurrentPosition(),
                        this.leftDriveFront.getCurrentPosition(),
                        this.rightDriveFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            this.stop();

            // Turn off RUN_TO_POSITION
            leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception ex){
            telemetry.addData("Issues running with encoders to position", ex);
            telemetry.update();
        }
    }

    public void encoderPivot(double speed,
                              double inches,
                              double timeoutS) {

        try {
            double val = Math.abs(inches);
            int newLeftTarget = 0 ;
            int newRightTarget = 0;
            int newLeftFrontTarget = 0;
            int newRightFrontTarget = 0;
            if (inches < 0){
                //pivot left
                newLeftTarget = this.leftDriveBack.getCurrentPosition() - (int) (val * COUNTS_PER_INCH_REV);
                newRightTarget = this.rightDriveBack.getCurrentPosition() + (int) (val * COUNTS_PER_INCH_REV);
                newLeftFrontTarget = this.leftDriveFront.getCurrentPosition() - (int) (val * COUNTS_PER_INCH_REV);
                newRightFrontTarget = this.rightDriveFront.getCurrentPosition() + (int) (val * COUNTS_PER_INCH_REV);
            }
            else{
                //pivot right
                newLeftTarget = this.leftDriveBack.getCurrentPosition() + (int) (val * COUNTS_PER_INCH_REV);
                newRightTarget = this.rightDriveBack.getCurrentPosition() - (int) (val * COUNTS_PER_INCH_REV);
                newLeftFrontTarget = this.leftDriveFront.getCurrentPosition() + (int) (val * COUNTS_PER_INCH_REV);
                newRightFrontTarget = this.rightDriveFront.getCurrentPosition() - (int) (val * COUNTS_PER_INCH_REV);
            }


            this.leftDriveBack.setTargetPosition(newLeftTarget);
            this.rightDriveBack.setTargetPosition(newRightTarget);
            this.leftDriveFront.setTargetPosition(newLeftFrontTarget);
            this.rightDriveFront.setTargetPosition(newRightFrontTarget);


            // Turn On RUN_TO_POSITION
            leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            this.leftDriveBack.setPower(Math.abs(speed));
            this.rightDriveBack.setPower(Math.abs(speed));
            this.leftDriveFront.setPower(Math.abs(speed));
            this.rightDriveFront.setPower(Math.abs(speed));

            boolean stop = false;
            while (!stop) {
                boolean timeUp = timeoutS > 0 && runtime.seconds() >= timeoutS;
                stop = timeUp || (!this.leftDriveBack.isBusy() || !this.rightDriveBack.isBusy()
                        || !this.leftDriveFront.isBusy() || !this.rightDriveFront.isBusy());
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Back: %7d :%7d front: %7d :%7d",
                        this.leftDriveBack.getCurrentPosition(),
                        this.rightDriveBack.getCurrentPosition(),
                        this.leftDriveFront.getCurrentPosition(),
                        this.rightDriveFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            this.stop();

            // Turn off RUN_TO_POSITION
            leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception ex){
            telemetry.addData("Issues running with encoders to position", ex);
            telemetry.update();
        }
    }


    public void moveTower(double drive){
        if (towerDrive != null ) {
            double power = Range.clip(drive, -1.0, 1.0);

            this.towerDrive.setPower(power);

            telemetry.addData("Tower", "Pos: %d, Speed from %.2f", this.towerDrive.getCurrentPosition(), drive);
        }
    }



    public void moveCrane(double drive){
        boolean extend = drive >= 0;
        if (craneDrive != null ) {
            cranePosition = this.craneDrive.getCurrentPosition();
            if (extend && cranePosition > MAX_CRANE_POS){
                this.craneDrive.setPower(0);
                return;
            }
            else if (extend && cranePosition >= MAX_CRANE_POS - 200){
                this.swivelStone(true);
            }
            else if (!extend && !isSwivel90() && cranePosition <=MAX_CRANE_POS*2/3){
                this.toggleStoneLock(false);
                this.swivelStone(false);
            }
//            if (!extend && cranePosition <= 0){
//                this.craneDrive.setPower(0);
//                return;
//            }
            double power = Range.clip(drive, -1.0, 1.0);

            this.craneDrive.setPower(power);

            telemetry.addData("Crane", "Pos %d, Drive: %.2f", this.craneDrive.getCurrentPosition(), drive);
        }
    }

//    public void pickup(float drive){
//        if (intake != null ) {
//            double power = Range.clip(drive, 0, 1.0);
//
//            this.intake.setPower(power);
//
//            telemetry.addData("Intake", "Speed from %.2f", drive);
//        }
//        else{
//            telemetry.addData("Intake", "Not initialized");
//        }
//    }
//
//    public void release(double drive, Telemetry telemetry){
//        if (intake != null ) {
//            double power = Range.clip(drive, -1, 0);
//
//            this.intake.setPower(power);
//
//            telemetry.addData("Intake", "Speed from %.2f", drive);
//        }
//    }


    public void positionCapstone(){
        if (capstone != null) {
            this.capstone.setPosition(0.63);
        }
        else{
            telemetry.addData("Capstone", "Not initialized");
        }
    }

    public void resetCapstone(){
        if (capstone != null) {
            this.capstone.setPosition(1);
        }
        else{
            telemetry.addData("Capstone", "Not initialized");
        }
    }

    public void toggleStoneLock(boolean lock){
        if (lockStone != null) {
            if (lock) {
                this.lockStone.setPosition(0.4);
            }
            else
            {
                this.lockStone.setPosition(0);
            }
        }
        else{
            telemetry.addData("lockStone", "Not initialized");
        }
    }

    public void swivelStone(boolean out){
        if (rotateStone != null) {
            if (out) {
                this.rotateStone.setPosition(0.7);
            }
            else
            {
                this.rotateStone.setPosition(0);
            }
        }
        else{
            telemetry.addData("rotateStone", "Not initialized");
        }
    }

    public void swivelStone90(){
        if (rotateStone != null) {
                this.rotateStone.setPosition(0.35);
        }
        else{
            telemetry.addData("rotateStone90", "Not initialized");
        }
    }

    public boolean isSwivel90(){
        return this.rotateStone.getPosition() >= 0.3 && this.rotateStone.getPosition() <=  0.4;
    }


    public void moveIntake(double drive){
        if (intakeLeft != null && intakeRight != null) {
            intakeLeft.setDirection(DcMotor.Direction.REVERSE);
            intakeRight.setDirection(DcMotor.Direction.FORWARD);
            double power = Range.clip(drive, 0, 0.5);
            intakeLeft.setPower(power);
            intakeRight.setPower(power);
        }
    }

    public void moveIntakeReverse(double drive){
        if (intakeLeft != null && intakeRight != null) {
            intakeLeft.setDirection(DcMotor.Direction.FORWARD);
            intakeRight.setDirection(DcMotor.Direction.REVERSE);
            double power = Range.clip(drive, 0, 0.5);
            intakeLeft.setPower(power);
            intakeRight.setPower(power);
        }
    }

    private double normalizeRange(double raw){
        if (raw == DistanceUnit.infinity) {
            raw = -1;
        }
        return raw;
    }


    public double getRangetoObstacleBack(){
        if (rangeBack == null) {
            return -1;
        }
        double range = rangeBack.getDistance(DistanceUnit.INCH);
        return normalizeRange(range);
    }

    public double getRangetoObstacleLeft(){
        if (rangeLeft == null) {
            return -1;
        }
        double range = rangeLeft.getDistance(DistanceUnit.INCH);
        return normalizeRange(range);
    }

    public double getRangetoObstacleRight(){
        if (rangeRight == null) {
            return -1;
        }
        double range = rangeRight.getDistance(DistanceUnit.INCH);
        return normalizeRange(range);
    }

    public double getStoneSensorData(){
        if (stoneSensor == null) {
            return -1;
        }
        double range = stoneSensor.getDistance(DistanceUnit.INCH);
        return normalizeRange(range);
    }

    public boolean isStoneInside(){
        double reading = getStoneSensorData();
        telemetry.addData("Stone Check", "Dist: %.2f, %b", reading, stoneSensor != null);
        return reading > 0 && reading <= 4;
    }

    public boolean autoLockStone(){
        boolean locked = false;
        boolean stoneInside = this.isStoneInside();
        if (stoneInside && cranePosition <= MAX_CRANE_POS/10){
            this.toggleStoneLock(true);
            locked = true;
        }
        return locked;
    }


//    public double getRangetoObstacleFrontLeft(){
//        if (rangeFrontLeft == null) {
//            return 0;
//        }
//        double range = rangeFrontLeft.getDistance(DistanceUnit.INCH);
//        return normalizeRange(range);
//    }
//
//    public double getRangetoObstacleFrontRight(){
//        if (rangeFrontRight == null) {
//            return 0;
//        }
//        double range = rangeFrontRight.getDistance(DistanceUnit.INCH);
//        return normalizeRange(range);
//    }



    public Gyro getGyro() {
        return gyro;
    }

    public void setGyro(Gyro gyro) {
        this.gyro = gyro;
    }

    public double encoderMoveDetect(double speed,
                                    double leftInches, double rightInches,
                                    double timeoutS, LinearOpMode caller, DetectionInterface callback) {

        double diff = 0;
        try {
            if (callback == null){
                return diff;
            }
            int start = this.leftDriveBack.getCurrentPosition();
            // Determine new target position, and pass to motor controller
            int newLeftTarget = this.leftDriveBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH_REV);
            int newRightTarget = this.rightDriveBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH_REV);
            int newLeftFrontTarget = this.leftDriveFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH_REV);
            int newRightFrontTarget = this.rightDriveFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH_REV);

            this.leftDriveBack.setTargetPosition(newLeftTarget);
            this.rightDriveBack.setTargetPosition(newRightTarget);
            this.leftDriveFront.setTargetPosition(newLeftFrontTarget);
            this.rightDriveFront.setTargetPosition(newRightFrontTarget);


            // Turn On RUN_TO_POSITION
            leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            this.leftDriveBack.setPower(Math.abs(speed));
            this.rightDriveBack.setPower(Math.abs(speed));
            this.leftDriveFront.setPower(Math.abs(speed));
            this.rightDriveFront.setPower(Math.abs(speed));

            boolean stop = false;
            boolean leftMove = leftInches != 0;
            boolean rightMove = rightInches != 0;
            boolean detected = false;
            while (!stop) {
                boolean timeUp = timeoutS > 0 && runtime.seconds() >= timeoutS;
                stop = !caller.opModeIsActive() || timeUp || ((leftMove && !this.leftDriveBack.isBusy()) || (rightMove && !this.rightDriveBack.isBusy())
                        || (leftMove && !this.leftDriveFront.isBusy()) || (rightMove && !this.rightDriveFront.isBusy()));
                detected = callback.detect();
                if (detected){
                    break;
                }
            }

            this.stop();
            int finish = this.leftDriveBack.getCurrentPosition();
            diff = (Math.abs(finish) - Math.abs(start))/COUNTS_PER_INCH_REV;


            // Turn off RUN_TO_POSITION
            leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception ex){
            telemetry.addData("Issues running with encoders to position", ex);
            telemetry.update();
        }

        return diff;
    }


    public void hookTray(boolean lock){
        if (hookRight != null && hookLeft != null) {
            if (lock) {
                this.hookRight.setPosition(0.2);
                this.hookLeft.setPosition(0.8);
            }
            else
            {
                this.hookRight.setPosition(1);
                this.hookLeft.setPosition(0);
            }
        }
        else{
            telemetry.addData("hookTray", "Not initialized");
        }
    }

    public void hookTraySide(boolean lock, boolean left){
        if (hookRight != null && hookLeft != null) {
            if (lock) {
                if (left){
                    this.hookLeft.setPosition(1);
                }
                else {
                    this.hookRight.setPosition(0);
                }

            }
            else
            {
                if (left){
                    this.hookLeft.setPosition(0);
                }
                else{
                    this.hookRight.setPosition(1);
                }
            }
        }
        else{
            telemetry.addData("hookTray", "Not initialized");
        }
    }


    public int preMoveCrane(double speed, double inches){
        int start = this.craneDrive.getCurrentPosition();
        int newTarget = start + (int) (inches * COUNTS_PER_MOTOR_CRANE);

        int diff = Math.abs(start - newTarget);
        int halfWay = start + diff/2;


        this.craneDrive.setTargetPosition(newTarget);


        // Turn On RUN_TO_POSITION
        craneDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // reset the timeout time and start motion.
        runtime.reset();
        this.craneDrive.setPower(Math.abs(speed));
        return halfWay;

    }

    public void postMoveCrane(){
        this.craneDrive.setPower(0);
        telemetry.addData("Motors", "Stopped");
        telemetry.update();

        // Turn off RUN_TO_POSITION
        craneDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderCrane(double speed, double inches, double timeoutS) {

        try {

            preMoveCrane(speed, inches);

            boolean stop = false;
            boolean move = inches != 0;
            while (!stop) {
                boolean timeUp = timeoutS > 0 && runtime.seconds() >= timeoutS;
                stop = timeUp || move && !this.craneDrive.isBusy();
            }

            postMoveCrane();
        }
        catch (Exception ex){
            telemetry.addData("Issues running with encoders to position", ex);
            telemetry.update();
        }
    }

    public boolean craneExtended(){
        cranePosition = this.craneDrive.getCurrentPosition();
        telemetry.addData("Crane pos", cranePosition);
        telemetry.update();
        return !this.craneDrive.isBusy() || cranePosition >= (MAX_CRANE_POS -200);
    }

    public void align(double toWall, double limit, double longCat, boolean redWall,  LinearOpMode caller){
        if (toWall > 0) {
            double range = 3;
//            if (redWall){
//                range = 3;
//            }
            double low = limit - range;
            double high = limit + range;
            int head = this.getGyro().getDesiredHeading();
            //wall too close
            if(toWall < low){
                double catet = limit - toWall;
                double travel = Math.sqrt(longCat*longCat + catet * catet);
                double t = catet/longCat;
                double rads = Math.atan(t);
                double degrees = Math.toDegrees(rads);
                if (redWall){
                    degrees = head + degrees;
                }
                else{
                    degrees = head - degrees;
                }
                this.getGyro().turn((int)degrees, 0.5, caller);
                this.encoderDrive(0.5, travel, travel,0,  caller);
                this.getGyro().turn(head, 0.4, caller);
            }
            //wall too far
            if (toWall > high){
                double catet = toWall - limit;
                double travel = Math.sqrt(longCat*longCat + catet * catet);
                double t = catet/longCat;
                double rads = Math.atan(t);
                double degrees = Math.toDegrees(rads);
                if (redWall){
                    degrees = head - degrees;
                }
                else{
                    degrees = head + degrees;
                }
                this.getGyro().turn((int)degrees, 0.5, caller);
                this.encoderDrive(0.5, travel, travel,0, caller);
                this.getGyro().turn(head, 0.4, caller);
            }
        }
    }

    public double curveToPath(int far, int close, double toWall, LinearOpMode caller, boolean dryrun){
        double travel = -1;
        int middle = close + (far - close)/2;
        int head = this.getGyro().getDesiredHeading();
        double distanceReductionClose  = GameStats.ROBOT_SIDE*2/3;
        double distanceReductionFar = GameStats.ROBOT_SIDE;
        double longCat = GameStats.TILE_WIDTH;
        if(toWall > far){
            double catet = toWall - middle;
            travel = Math.sqrt(longCat*longCat + catet * catet);
            double t = catet/longCat;
            double rads = Math.atan(t);
            double degrees =  Math.toDegrees(rads);
            if (!dryrun) {
                this.getGyro().pivotForward((int)(degrees + head), -0.8, caller);
                stop();
                encoderDrive(-0.8, -(travel - distanceReductionFar), -(travel - distanceReductionFar), 0, caller);
                this.getGyro().pivotForward(head + 5, -0.8, caller);
            }
            this.stop();

        }
        else if (toWall < close){
            double catet = middle - toWall;
            travel = Math.sqrt(longCat*longCat + catet * catet);
            double t = catet/longCat;
            double rads = Math.atan(t);
            double degrees =  Math.toDegrees(rads);

            if (!dryrun) {
                this.getGyro().pivotForward((int)(head - degrees), -0.8, caller);
                stop();
                telemetry.addData("Current", getGyro().getHeading());
                telemetry.addData("Desired", head);
                encoderDrive(-0.8, -(travel - distanceReductionClose), -(travel - distanceReductionClose), 0, caller);
                this.getGyro().pivotForward(head - 5 , -0.8, caller);
            }
            this.stop();

        }
        telemetry.addData("Travel", travel);
        telemetry.addData("Middle", middle);
        telemetry.update();
        return travel;
    }

    public double curveToPathReverse(int far, int close, double toWall, LinearOpMode caller, boolean dryrun){
        double travel = -1;
        int middle = close + (far - close)/2;
        int head = this.getGyro().getDesiredHeading();
        double distanceReduction  = GameStats.ROBOT_SIDE*2/3;
        double longCat = GameStats.TILE_WIDTH;
        if(toWall > far){
            double catet = toWall - middle;
            travel = Math.sqrt(longCat*longCat + catet * catet);
            double t = catet/longCat;
            double rads = Math.atan(t);
            double degrees =  Math.toDegrees(rads);

            if (!dryrun) {
                this.getGyro().pivot((int)(head - degrees), -0.8, caller);
                stop();
                telemetry.addData("Current", getGyro().getHeading());
                telemetry.addData("Desired", head);
                encoderDrive(-0.7, -(travel - distanceReduction), -(travel - distanceReduction), 0, caller);
                this.getGyro().pivot(head, -0.8, caller);
            }
            this.stop();

        }
        else if (toWall < close){
            double catet = middle - toWall;
            travel = Math.sqrt(longCat*longCat + catet * catet);
            double t = catet/longCat;
            double rads = Math.atan(t);
            double degrees =  Math.toDegrees(rads);
            if (!dryrun) {
                this.getGyro().pivot((int)(degrees + head), -0.7, caller);
                stop();
                encoderDrive(-0.7, -(travel - distanceReduction), -(travel - distanceReduction), 0, caller);
                this.getGyro().pivot(head, -0.8, caller);
            }

            this.stop();

        }
        telemetry.addData("Travel", travel);
        telemetry.addData("Middle", middle);
        telemetry.update();
        return travel;
    }
}
