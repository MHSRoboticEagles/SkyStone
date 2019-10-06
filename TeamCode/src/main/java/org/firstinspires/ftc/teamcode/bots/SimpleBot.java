package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by sjeltuhin on 9/12/17.
 */

public class SimpleBot {
    public DcMotor leftDriveBack = null;
    public DcMotor rightDriveBack = null;

    public DcMotor leftDriveFront = null;
    public DcMotor rightDriveFront = null;



    private ElapsedTime     runtime = new ElapsedTime();



    private double ANTI_GRAVITY_POWER = 0.01;

    public double DRIVE_SPEED = 0.99;

    static final double     STRAFE_INCH    = 1.5 ;    // Rev Core Hex motor



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
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDriveBack  = hwMap.get(DcMotor.class, "left_drive_back");
        rightDriveBack = hwMap.get(DcMotor.class, "right_drive_back");
        leftDriveFront  = hwMap.get(DcMotor.class, "left_drive_front");
        rightDriveFront = hwMap.get(DcMotor.class, "right_drive_front");

        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);
        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotor.Direction.FORWARD);

        resetEncoders();

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

    protected void resetEncoders(){
        leftDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void initMode(DcMotor.RunMode mode){
        leftDriveBack.setMode(mode);
        rightDriveBack.setMode(mode);
        leftDriveFront.setMode(mode);
        rightDriveFront.setMode(mode);
    }

    public void stop (){
        this.leftDriveBack.setPower(0);
        this.rightDriveBack.setPower(0);
        this.leftDriveFront.setPower(0);
        this.rightDriveFront.setPower(0);
    }

    public void move(double drive, double turn, Telemetry telemetry){
        double rightPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        double leftPower   = Range.clip(drive - turn, -1.0, 1.0) ;
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


    public void strafeLeft(double speed, Telemetry telemetry){
        double power    = Range.clip(speed, -1.0, 1.0) ;
        this.leftDriveBack.setPower(power);
        this.rightDriveBack.setPower(-power);
        this.leftDriveFront.setPower(-power);
        this.rightDriveFront.setPower(power);
        telemetry.addData("Motors", "Front: %.0f", power);
        telemetry.addData("Motors", "Back: %.0f", power);
    }

    public void strafeRight(double speed, Telemetry telemetry){
        double power    = Range.clip(speed, -1.0, 1.0) ;
        this.leftDriveBack.setPower(-power);
        this.rightDriveBack.setPower(power);
        this.leftDriveFront.setPower(power);
        this.rightDriveFront.setPower(-power);
        telemetry.addData("Motors", "Front: %.0f", power);
        telemetry.addData("Motors", "Back: %.0f", power);
    }

    public void pivotLeft(double speed, Telemetry telemetry){
        double power    = Range.clip(speed, -1.0, 1.0) ;
        this.leftDriveBack.setPower(-power);
        this.rightDriveBack.setPower(power);
        this.leftDriveFront.setPower(-power);
        this.rightDriveFront.setPower(power);
        telemetry.addData("Motors", "Left: %7d Right: %7d", leftDriveFront.getCurrentPosition(), rightDriveFront.getCurrentPosition());
        telemetry.update();
    }

    public void pivotRight(double speed, Telemetry telemetry){
        double power    = Range.clip(speed, -1.0, 1.0) ;
        this.leftDriveBack.setPower(power);
        this.rightDriveBack.setPower(-power);
        this.leftDriveFront.setPower(power);
        this.rightDriveFront.setPower(-power);
        telemetry.addData("Motors", "Lef: %7d Right: %7d", leftDriveFront.getCurrentPosition(), rightDriveFront.getCurrentPosition());
        telemetry.update();
    }

    public void turnLeft(double speed){
        this.leftDriveBack.setPower(0);
        this.rightDriveBack.setPower(speed);
        this.leftDriveFront.setPower(0);
        this.rightDriveFront.setPower(speed);
    }

    public void turnRight(double speed){
        this.leftDriveBack.setPower(speed);
        this.rightDriveBack.setPower(0);
        this.leftDriveFront.setPower(speed);
        this.rightDriveFront.setPower(0);
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

    public void encoderStrafe(double speed,
                             double distanceInches,
                             double timeoutS, Telemetry telemetry) {

        try {
            double val = Math.abs(distanceInches);
            int newLeftTarget = 0 ;
            int newRightTarget = 0;
            int newLeftFrontTarget = 0;
            int newRightFrontTarget = 0;
            int increment = (int) (val * COUNTS_PER_INCH_REV * STRAFE_INCH);;
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

}
