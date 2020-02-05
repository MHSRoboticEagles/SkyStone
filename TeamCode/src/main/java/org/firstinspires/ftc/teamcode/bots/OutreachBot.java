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

public class OutreachBot {
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor arm = null;
    public Servo grabber = null;

    private ElapsedTime runtime = new ElapsedTime();

    double grabPos = -1;

    /* local OpMode members. */
    HardwareMap hwMap =  null;

    /* Constructor */
    public OutreachBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry telemetry) throws Exception {
        // Save reference to Hardware map
        hwMap = ahwMap;

        try {
            // Define and Initialize Motors
            leftDrive = hwMap.get(DcMotor.class, "left_drive");
            rightDrive = hwMap.get(DcMotor.class, "right_drive");
            telemetry.addData("Init", "Drive");
        } catch (Exception ex) {
            //issues accessing drive resources
            throw new Exception("Issues accessing drive resources. Check the controller config", ex);
        }

        try{
            arm = hwMap.get(DcMotor.class, "arm");
        }
        catch (Exception ex) {
            throw new Exception("Issues accessing arm. Check the controller config", ex);
        }

        if (leftDrive != null) {
            leftDrive.setDirection(DcMotor.Direction.FORWARD);
        }

        if (rightDrive != null) {
            rightDrive.setDirection(DcMotor.Direction.REVERSE);
        }

        if (leftDrive != null && rightDrive != null) {

            // Set all motors to zero power
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }

        if (arm != null){
            arm.setDirection(DcMotor.Direction.REVERSE);
            arm.setPower(0);
        }

        // Servos

        try{
            grabber = hwMap.get(Servo.class, "grabber");
            if (grabber != null){
                grabPos = this.grabber.getPosition();
            }
        }
        catch (Exception ex){
            throw new Exception("Issues accessing grabbing servo. Check the controller config", ex);
        }

        this.stop();
    }

    public void initMode (DcMotor.RunMode mode) {
        if (leftDrive != null && rightDrive != null) {
            leftDrive.setMode(mode);
            rightDrive.setMode(mode);
        }
    }

    private void stop() {
        if (leftDrive != null && rightDrive != null) {
            this.leftDrive.setPower(0);
            this.rightDrive.setPower(0);
        }
    }

    public void move(double drive) {
        if (leftDrive != null && rightDrive != null) {
            double rightPower = Range.clip(drive, -1.0, 1.0);
            double leftPower = Range.clip(drive, -1.0, 1.0);

            this.leftDrive.setPower(leftPower);
            this.rightDrive.setPower(rightPower);
        }
    }

    public void pivotLeft(double speed) {
        if (leftDrive != null && rightDrive != null) {
            double power = Range.clip(speed, -1.0, 1.0);
            this.leftDrive.setPower(-power);
            this.rightDrive.setPower(power);
        }
    }

    public void pivotRight(double speed) {
        if (leftDrive != null && rightDrive != null) {
            double power = Range.clip(speed, -1.0, 1.0);
            this.leftDrive.setPower(power);
            this.rightDrive.setPower(-power);
        }
    }

    public void moveArm(double degrees){
        if (arm != null ) {
            double power = Range.clip(degrees,-1.0,1.0);

            this.arm.setPower(power);

        }
    }

    public void lock(){
        if (grabber != null) {
            this.grabber.setPosition(0.63);
        }
    }

    public void unlock(){
        if (grabber != null) {
            this.grabber.setPosition(1);
        }
    }

}
