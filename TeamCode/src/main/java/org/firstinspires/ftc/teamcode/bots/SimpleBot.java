package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.skills.DetectionInterface;
import org.firstinspires.ftc.teamcode.skills.Gyro;

/**
 * Created by sjeltuhin on 9/12/17.
 */

public class SimpleBot {
    public DcMotor leftDriveBack = null;
    public DcMotor rightDriveBack = null;

    public DcMotor leftDriveFront = null;
    public DcMotor rightDriveFront = null;

    public DcMotor towerDrive = null;
    public DcMotor towerDrive2 = null;

    public DcMotor platformDrive = null;

//    public DcMotor intake = null;

    public Servo intakeTemp = null;

    public Servo capstone = null;

    public DcMotor intakePivot = null;

    private Gyro gyro = null;

    private DistanceSensor sensorRange;



    private ElapsedTime     runtime = new ElapsedTime();



    public double ANTI_GRAVITY_POWER = -0.5;

    public double DRIVE_SPEED = 0.99;

    static final double     STRAFE_COUNT_INCH    =  20;    // Rev Core Hex motor
    public static final double     PIVOT_SPEED = 0.2;

    public static final double     INTAKE_PIVOT_SPEED = 1;
    public static final double     INTAKE_PIVOT_SPEED_UP = 1;



    //REV

    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // Rev Core Hex motor
    static final double     DRIVE_GEAR_REDUCTION    = 0.517 ;     // This is < 1.0 if geared UP. was 2 in the sample
    static final double     WHEEL_DIAMETER_INCHES   = 4.05 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH_REV     = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);  //22.64


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public SimpleBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry telemetry) throws Exception{
        // Save reference to Hardware map
        hwMap = ahwMap;

        try {

            // Define and Initialize Motors
            leftDriveBack = hwMap.get(DcMotor.class, "left_drive_back");
            rightDriveBack = hwMap.get(DcMotor.class, "right_drive_back");
            leftDriveFront = hwMap.get(DcMotor.class, "left_drive_front");
            rightDriveFront = hwMap.get(DcMotor.class, "right_drive_front");
        }
        catch (Exception ex){
            //issues accessing drive resources
            throw new Exception("Issues accessing drive resources. Check the controller config", ex);
        }

        try{
            towerDrive = hwMap.get(DcMotor.class, "tower");
        }
        catch (Exception ex){
            throw new Exception("Issues accessing tower. Check the controller config", ex);
        }
        telemetry.addData("Init", "towerDrive");

        try{
            towerDrive2 = hwMap.get(DcMotor.class, "second_lift");
        }
        catch (Exception ex){
            throw new Exception("Issues accessing tower drive 2. Check the controller config", ex);
        }
        telemetry.addData("Init", "towerDrive2");

        try{
            platformDrive = hwMap.get(DcMotor.class, "platform");
        }
        catch (Exception ex){
            throw new Exception("Issues accessing platform. Check the controller config", ex);
        }
        telemetry.addData("Init", "platformDrive");

//        try{
//            intake = hwMap.get(DcMotor.class, "intake");
//        }
//        catch (Exception ex){
//            throw new Exception("Issues accessing intake. Check the controller config", ex);
//        }
//        telemetry.addData("Init", "intake");


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
            towerDrive.setDirection(DcMotor.Direction.FORWARD);
            towerDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            towerDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            towerDrive.setPower(0);
        }

        if (towerDrive2 != null){
            towerDrive2.setDirection(DcMotor.Direction.FORWARD);
            towerDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            towerDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            towerDrive2.setPower(0);
        }

        if (platformDrive != null){
            platformDrive.setDirection(DcMotor.Direction.REVERSE);
            platformDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            platformDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            platformDrive.setPower(0);
        }

//        if (intake != null){
//            intake.setDirection(DcMotor.Direction.REVERSE);
//            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            intake.setPower(0);
//        }

        try{
            intakePivot = hwMap.get(DcMotor.class, "pivot");
            if (intakePivot != null){
                intakePivot.setDirection(DcMotor.Direction.FORWARD);
                intakePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                intakePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                intakePivot.setPower(0);
            }
        }
        catch (Exception ex){
            throw new Exception("Issues accessing intakePivot servo. Check the controller config", ex);
        }

        try{
            intakeTemp = hwMap.get(Servo.class, "temp-intake");
            if (intakeTemp != null){
                intakeTemp.setPosition(1);
            }
        }
        catch (Exception ex){
            throw new Exception("Issues accessing temp intake servo. Check the controller config", ex);
        }

        try{
            capstone = hwMap.get(Servo.class, "capstone_servo");
            if (capstone != null){
                capstone.setPosition(1);
            }
        }
        catch (Exception ex){
            throw new Exception("Issues accessing temp intake servo. Check the controller config", ex);
        }


        try{
            sensorRange = hwMap.get(DistanceSensor.class, "sensor_range");
        }
        catch (Exception ex){
            throw new Exception("Issues accessing range sensor. Check the controller config", ex);
        }

        setGyro(new Gyro());
        getGyro().init(this, hwMap, telemetry);

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

    public void move(double drive, double turn, Telemetry telemetry){
        if (leftDriveBack != null && rightDriveBack!= null && leftDriveFront != null && rightDriveFront != null) {
            double rightPower = Range.clip(drive + (turn * 0.85), -1.0, 1.0);
            double leftPower = Range.clip(drive - (turn * 0.85), -1.0, 1.0);
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


    public void strafeLeft(double speed, Telemetry telemetry){
        if (leftDriveBack != null && rightDriveBack!= null && leftDriveFront != null && rightDriveFront != null) {
            double power = Range.clip(speed, -1.0, 1.0);
            this.leftDriveBack.setPower(power);
            this.rightDriveBack.setPower(-power);
            this.leftDriveFront.setPower(-power);
            this.rightDriveFront.setPower(power);
            telemetry.addData("Motors", "Front: %.0f", power);
            telemetry.addData("Motors", "Back: %.0f", power);
        }
    }

    public void strafeRight(double speed, Telemetry telemetry){
        if (leftDriveBack != null && rightDriveBack!= null && leftDriveFront != null && rightDriveFront != null) {
            double power = Range.clip(speed, -1.0, 1.0);
            this.leftDriveBack.setPower(-power);
            this.rightDriveBack.setPower(power);
            this.leftDriveFront.setPower(power);
            this.rightDriveFront.setPower(-power);
            telemetry.addData("Motors", "Front: %.0f", power);
            telemetry.addData("Motors", "Back: %.0f", power);
        }
    }

    public void pivotLeft(double speed, Telemetry telemetry){
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

    public void pivotRight(double speed, Telemetry telemetry){
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

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS, Telemetry telemetry) {

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

    public void encoderTurn(double turn,
                             double leftInches, double rightInches,
                             double timeoutS, Telemetry telemetry) {

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

    public void encoderStrafe(double speed,
                             double distanceInches,
                             double timeoutS, Telemetry telemetry) {

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
                              double timeoutS, Telemetry telemetry) {

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


    public void moveTower(double drive, Telemetry telemetry){
        if (towerDrive != null ) {
            double power = Range.clip(drive, -1.0, 1.0);

            this.towerDrive.setPower(power);

            telemetry.addData("Tower", "Speed from %.2f", drive);
        }

        if (towerDrive2 != null ) {
            double power = Range.clip(drive, -1.0, 1.0);

            this.towerDrive2.setPower(power);

            telemetry.addData("Tower 2", "Speed from %.2f", drive);
        }
    }



    public void movePlatform(double drive, Telemetry telemetry){
        if (platformDrive != null ) {
            double power = Range.clip(drive, -1.0, 1.0);

            this.platformDrive.setPower(power);

            telemetry.addData("Platform", "Speed from %.2f", drive);
        }
    }

//    public void pickup(float drive, Telemetry telemetry){
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

    public void pickupTemp(float drive, Telemetry telemetry){
        if (intakeTemp != null && drive > 0) {
            this.intakeTemp.setPosition(0);

            telemetry.addData("Intake Temp", "Speed from %.2f", drive);
        }
        else{
            telemetry.addData("Intake Temp", "Not initialized");
        }
    }

    public void releaseTemp(double drive, Telemetry telemetry){
        if (intakeTemp != null && drive > 0 ) {
            this.intakeTemp.setPosition(1);

            telemetry.addData("Temp Intake", "Speed from %.2f", drive);
        }
        else{
            telemetry.addData("Temp Intake", "Not initialized");
        }
    }

    public void dropCapstone(boolean drop, Telemetry telemetry){
        if (capstone != null) {
            if (drop) {
                this.capstone.setPosition(0);
            }
            else
            {
                this.capstone.setPosition(1);
            }
        }
        else{
            telemetry.addData("Capstone", "Not initialized");
        }
    }

    public void fold (boolean move){
        if (intakePivot != null){
            if (move){
                intakePivot.setPower(-PIVOT_SPEED*3);
            }else {
                intakePivot.setPower(0);
            }
        }
    }

    public void unfold (boolean move){
        if (intakePivot != null){
            if (move){
                intakePivot.setPower(PIVOT_SPEED);
            }else {
                intakePivot.setPower(0);
            }
        }
    }

    public void moveIntake(double drive, Telemetry telemetry){
        if (intakePivot != null ) {
            double power = Range.clip(drive, -1.0, 1.0);

            this.intakePivot.setPower(power/3);

            telemetry.addData("Pivot", "Speed from %.2f", drive);
        }
    }

    public void encoderIntakeMove(int fold, double speed, double timeoutS, Telemetry telemetry) {

        try {
            // Determine new target position, and pass to motor controller
            int to = (int) (fold* COUNTS_PER_MOTOR_REV/4.5);
            if (fold > 0){
                to = (int) (fold* COUNTS_PER_MOTOR_REV/4);
            }
            int newTarget = this.intakePivot.getCurrentPosition() + to;


            this.intakePivot.setTargetPosition(newTarget);



            // Turn On RUN_TO_POSITION
            intakePivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            this.intakePivot.setPower(Math.abs(speed));


            boolean stop = false;

            while (!stop) {
                boolean timeUp = timeoutS > 0 && runtime.seconds() >= timeoutS;
                stop = timeUp || (!this.intakePivot.isBusy());
            }


            this.intakePivot.setPower(0);

            // Turn off RUN_TO_POSITION
            intakePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception ex){
            telemetry.addData("Issues running with encoders to position", ex);
            telemetry.update();
        }
    }

    public void intakePressDown(){
        this.intakePivot.setPower(ANTI_GRAVITY_POWER);
    }

    public double getRangetoObstacle(){
        if (sensorRange == null) {
            return 0;
        }
        double range = sensorRange.getDistance(DistanceUnit.INCH);
        return range;
    }

    public Gyro getGyro() {
        return gyro;
    }

    public void setGyro(Gyro gyro) {
        this.gyro = gyro;
    }

    public void encoderMoveDetect(double speed,
                              double leftInches, double rightInches,
                              double timeoutS, Telemetry telemetry, DetectionInterface callback) {

        try {
            if (callback == null){
                return;
            }
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
                stop = timeUp || detected || ((leftMove && !this.leftDriveBack.isBusy()) || (rightMove && !this.rightDriveBack.isBusy())
                        || (leftMove && !this.leftDriveFront.isBusy()) || (rightMove && !this.rightDriveFront.isBusy()));

                detected = callback.detect();

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


    //callbacks

}
