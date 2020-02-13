package org.firstinspires.ftc.teamcode.skills;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV1;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV2;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxFirmwareVersionManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.bots.TieBot;

public class Gyro {
//    LynxEmbeddedIMU imu;
    BNO055IMU imu;
    double                  globalAngle = 0, power = .30, correction;
    TieBot robot = null;
    Telemetry telemetry;
    Position lastPos = null;
    Velocity lastVelocity = null;
    private int desiredHeading = 0;
    int MIN_TOLERANCE = -2;
    int MAX_TOLERANCE = 2;
    private ElapsedTime runtime = new ElapsedTime();

    private static class SkipWinI2cDeviceSynchImplOnSimple extends I2cDeviceSynchImplOnSimple {
        private SkipWinI2cDeviceSynchImplOnSimple(I2cDeviceSynchSimple simple, boolean isSimpleOwned) {
            super(simple, isSimpleOwned);
        }

        @Override
        public void setReadWindow(ReadWindow window) {
            // intentionally do nothing
        }
    }

    private int getLynxI2cVersion(LynxModule module,  HardwareMap ahwMap) {
        LynxI2cDeviceSynch defaultLynxI2cDevice =
                LynxFirmwareVersionManager.createLynxI2cDeviceSynch(ahwMap.appContext, module, 0);
        if (defaultLynxI2cDevice instanceof LynxI2cDeviceSynchV1) {
            return 1;
        } else if (defaultLynxI2cDevice instanceof LynxI2cDeviceSynchV2) {
            return 2;
        } else {
            return -1;
        }
    }

    public void init(TieBot owner, HardwareMap ahwMap, Telemetry t){
        robot = owner;
        telemetry = t;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

//        LynxModule module = ahwMap.get(LynxModule.class, "imu");
//        int i2cVersion = getLynxI2cVersion(module, ahwMap);
//        if (i2cVersion == 1){
//            imu = new LynxEmbeddedIMU(new I2cDeviceSynchImplOnSimple(
//                    new LynxI2cDeviceSynchV1(ahwMap.appContext, module, 0), true));
//        }
//        else if (i2cVersion == 2){
//            new LynxEmbeddedIMU(new I2cDeviceSynchImplOnSimple(
//                    new LynxI2cDeviceSynchV2(ahwMap.appContext, module, 0), true));
//        }

        imu = (BNO055IMU) ahwMap.get("imu");
        if (imu != null){
            imu.initialize(parameters);
            // Start the logging of measured acceleration
//            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            telemetry.addData("Info", "Gyro initialized");
        }
        else{
            telemetry.addData("Error", "Gyro failed");
        }

    }

    public void recordAcceleration(){
        imu.startAccelerationIntegration(lastPos, lastVelocity, 50);
    }

    public void stopRecordingAcceleration(){
        imu.stopAccelerationIntegration();
    }

    public Orientation getOrientation()  {
        Orientation orient = imu.getAngularOrientation();
        double angle = orient.firstAngle;
        telemetry.addData("1 imu heading", angle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("3 correction", correction);
        telemetry.addData("Mode", imu.getParameters().mode);
        return orient;
    }

    public double getHeading()  {
        Orientation orient = imu.getAngularOrientation();
        double angle = orient.firstAngle;

        return angle;
    }

    public Position getPosition()  {

        Position prevPos = lastPos;

        lastPos = imu.getPosition();
        lastVelocity = imu.getVelocity();

        if (lastPos != null) {
            double x = lastPos.x;
            double y = lastPos.y;
            double z = lastPos.z;
            telemetry.addData("X", x);
            telemetry.addData("Y", y);
            telemetry.addData("z", z);

            if (prevPos != null ){
                telemetry.addData("Diff X", Math.abs(x-prevPos.x));
                telemetry.addData("Diff Y", Math.abs(y-prevPos.y));
                telemetry.addData("Diff z", Math.abs(z-prevPos.z));
            }
        }

        return lastPos;
    }

    public void correct(){
        double degrees = checkDirection();
        telemetry.addData("Angle to correct", degrees);
        rotate((int)degrees, 0.3);
    }
    public void correct(double speed){
        double degrees = checkDirection();
        telemetry.addData("Angle to correct", degrees);
        rotate((int)degrees, speed);
    }

    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

//        correction = correction * gain;

        return correction;
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation();//imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("Current angle", angles.firstAngle);
        telemetry.addData("Desired angle", globalAngle);
        double deltaAngle = angles.firstAngle - globalAngle;
        telemetry.addData("deltaAngle", deltaAngle);
        telemetry.update();

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

//        globalAngle += deltaAngle;
//
//        lastAngles = angles;

        return deltaAngle; //globalAngle;
    }

    public void turn(int degrees, double power, LinearOpMode caller) {
        turn(degrees, power, 0, caller);
    }

    public void turn(int degrees, double power, int timeoutMS, LinearOpMode caller){

        //set to 0 heading first
//        correct();
        int lastDesiredHeading = desiredHeading;
        desiredHeading = degrees;
        int current = lastDesiredHeading; //(int)this.getHeading();
        double  leftPower = 0, rightPower = 0;

        boolean left = false;
        boolean right = false;
        double cutOff = 0;
        if (getDesiredHeading() > current){
            //turn left
            left = true;
            cutOff = getDesiredHeading() - (getDesiredHeading() - current)/5;
        }
        else if (getDesiredHeading() < current){
            right = true;
            cutOff = getDesiredHeading() + (current - getDesiredHeading())/5;
        }
        else{
            return;
        }

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        double minLeftSpeed = 0.3;
        double minRighSpeed = 0.3;

        if (right)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
            minLeftSpeed = -minLeftSpeed;
        }
        else if (left)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
            minRighSpeed = -minRighSpeed;
        }
        else return;

        // set power to rotate.
        robot.leftDriveBack.setPower(leftPower);
        robot.leftDriveFront.setPower(leftPower);
        robot.rightDriveBack.setPower(rightPower);
        robot.rightDriveFront.setPower(rightPower);

        runtime.reset();

        while (true){
            current = (int)this.getHeading();
            if (lastDesiredHeading > 0 && current < 0){
                continue;
            }
            telemetry.addData("current", current);
            telemetry.addData("desired", getDesiredHeading());
            telemetry.update();
            if (current >= cutOff - MAX_TOLERANCE && current <= cutOff + MAX_TOLERANCE){
                //drop the speed
                robot.leftDriveBack.setPower(minLeftSpeed);
                robot.leftDriveFront.setPower(minLeftSpeed);
                robot.rightDriveBack.setPower(minRighSpeed);
                robot.rightDriveFront.setPower(minRighSpeed);
            }
            if (timeoutMS > 0 && timeoutMS <= runtime.milliseconds()){
                break;
            }
            if (!caller.opModeIsActive() || (right && current <= getDesiredHeading())
                    || (left && (current >= getDesiredHeading() || (getDesiredHeading() == 180 && current < 0)))){
                break;
            }
        }

        robot.leftDriveBack.setPower(0);
        robot.leftDriveFront.setPower(0);
        robot.rightDriveBack.setPower(0);
        robot.rightDriveFront.setPower(0);
    }

    public void turnAndExtend(int degrees, double power, boolean extend, LinearOpMode caller){
        //set to 0 heading first
//        correct();
        desiredHeading = degrees;
        int current = (int)this.getHeading();
        double  leftPower = 0, rightPower = 0;

        boolean left = false;
        boolean right = false;
        double cutOff = 0;
        if (getDesiredHeading() > current){
            //turn left
            left = true;
            cutOff = getDesiredHeading() - (getDesiredHeading() - current)/5;
        }
        else if (getDesiredHeading() < current){
            right = true;
            cutOff = getDesiredHeading() + (current - getDesiredHeading())/5;
        }
        else{
            return;
        }

        // restart imu movement tracking.
        resetAngle();


        double minLeftSpeed = 0.3;
        double minRighSpeed = 0.3;

        if (right)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
            minLeftSpeed = -minLeftSpeed;
        }
        else if (left)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
            minRighSpeed = -minRighSpeed;
        }
        else return;

        // set power to rotate.
        robot.leftDriveBack.setPower(leftPower);
        robot.leftDriveFront.setPower(leftPower);
        robot.rightDriveBack.setPower(rightPower);
        robot.rightDriveFront.setPower(rightPower);

        if (extend) {
            robot.preMoveCrane(1, 10);
        }

        boolean turnDone = false;

        while (!turnDone){
            if (!turnDone) {
                current = (int) this.getHeading();
                telemetry.addData("current", current);
                telemetry.addData("desired", getDesiredHeading());
                telemetry.update();
                if (current >= cutOff - MAX_TOLERANCE && current <= cutOff + MAX_TOLERANCE) {
                    //drop the speed
                    robot.leftDriveBack.setPower(minLeftSpeed);
                    robot.leftDriveFront.setPower(minLeftSpeed);
                    robot.rightDriveBack.setPower(minRighSpeed);
                    robot.rightDriveFront.setPower(minRighSpeed);
                }
                if (!caller.opModeIsActive() || (right && current <= getDesiredHeading())
                        || (left && (current >= getDesiredHeading() || (getDesiredHeading() == 180 && current < 0)))) {
                    turnDone = true;
                    robot.stop();
                }
            }
        }

        robot.stop();
    }

    public void turnBackReverse(int degrees, double power, LinearOpMode caller){
        //set to 0 heading first
//        correct();
        desiredHeading = degrees;
        double  leftPower = 0, rightPower = 0;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        robot.leftDriveBack.setPower(leftPower);
        robot.leftDriveFront.setPower(leftPower);
        robot.rightDriveBack.setPower(rightPower);
        robot.rightDriveFront.setPower(rightPower);


        while (true){
            int current = (int)this.getHeading();
            telemetry.addData("current", current);
            telemetry.addData("desired", getDesiredHeading());
            telemetry.update();
            if (!caller.opModeIsActive() || (degrees < 0 && current >= getDesiredHeading())
                    || (degrees > 0 && current <= getDesiredHeading())){
                break;
            }
        }

        robot.leftDriveBack.setPower(0);
        robot.leftDriveFront.setPower(0);
        robot.rightDriveBack.setPower(0);
        robot.rightDriveFront.setPower(0);
    }


    public void pivot(int degrees, double power, LinearOpMode caller){
//        correct();
        desiredHeading = degrees;
        double  leftPower = 0, rightPower = 0;
        int current = (int)this.getHeading();
        boolean left = false;
        boolean right = false;

        if (desiredHeading < current){
            right= true;
        } else if(desiredHeading >current){
            left = true;

        } else {
            return;
        }


        if (right)
        {   // turn right.
            rightPower = power;
            robot.rightDriveBack.setPower(rightPower);
            robot.rightDriveFront.setPower(rightPower);

            robot.leftDriveBack.setPower(leftPower);
            robot.leftDriveFront.setPower(leftPower);
        }
        else if (left)
        {   // turn left.
            leftPower = power;
            robot.leftDriveBack.setPower(leftPower);
            robot.leftDriveFront.setPower(leftPower);

            robot.rightDriveBack.setPower(rightPower);
            robot.rightDriveFront.setPower(rightPower);
        }
        else return;

        // set power to rotate.



        while (true){
            current = (int)this.getHeading();
            telemetry.addData("current", current);
            telemetry.addData("desired", getDesiredHeading());
            telemetry.update();
            if (!caller.opModeIsActive() || (right && current <= getDesiredHeading())
                    || (left && current >= getDesiredHeading())){
                break;
            }
        }
        robot.stop();


    }

    public void pivotForward(int degrees, double power, LinearOpMode caller){
//        correct();
        desiredHeading = degrees;
        double  leftPower = 0, rightPower = 0;

        int current = (int)this.getHeading();

        boolean left = false;
        boolean right = false;

        if (getDesiredHeading() > current){
            right = true;

        }
        else if (getDesiredHeading() < current){
            left = true;
        }
        else{
            return;
        }


        if (right)
        {   // turn right.
            rightPower = power;
            robot.rightDriveBack.setPower(rightPower);
            robot.rightDriveFront.setPower(rightPower);
        }
        else if (left)
        {   // turn left.
            leftPower = power;
            robot.leftDriveBack.setPower(leftPower);
            robot.leftDriveFront.setPower(leftPower);
        }
        else return;

        // set power to rotate.



        while (true){
            current = (int)this.getHeading();
            telemetry.addData("current", current);
            telemetry.addData("desired", getDesiredHeading());
            telemetry.update();
            if (!caller.opModeIsActive() || (left && current <= getDesiredHeading())
                    || (right && current >= getDesiredHeading())){
                break;
            }
        }

    }


    public void pivotReverse(int degrees, double power, LinearOpMode caller){
//        correct();
        desiredHeading = degrees;
        double  leftPower = 0, rightPower = 0;
        int current = (int)this.getHeading();

        boolean left = false;
        boolean right = false;

        if (getDesiredHeading() > current){
            right = true;

        }
        else if (getDesiredHeading() < current){
            left = true;
        }
        else{
            return;
        }


        if (right)
        {   // turn right.
            rightPower = power;
            robot.rightDriveBack.setPower(rightPower);
            robot.rightDriveFront.setPower(rightPower);
        }
        else if (left)
        {   // turn left.
            leftPower = power;
            robot.leftDriveBack.setPower(leftPower);
            robot.leftDriveFront.setPower(leftPower);
        }
        else return;

        // set power to rotate.



        while (true){
            current = (int)this.getHeading();
            telemetry.addData("current", current);
            telemetry.addData("desired", getDesiredHeading());
            telemetry.update();
            if (!caller.opModeIsActive() || (left && current <= getDesiredHeading())
                    || (right && current >=  getDesiredHeading())){
                break;
            }
        }

    }

    public void pivotReverse(int degrees, double power, double subpower, LinearOpMode caller){
//        correct();
        desiredHeading = degrees;
        double  leftPower = 0, rightPower = 0;

        // restart imu movement tracking.
        resetAngle();

        if (power > 0){
            subpower = -subpower;
        }

        if (degrees > 0)
        {   // turn right.
            rightPower = power;

            robot.rightDriveBack.setPower(rightPower);
            robot.rightDriveFront.setPower(rightPower);

            robot.leftDriveBack.setPower(subpower);
            robot.leftDriveFront.setPower(subpower);
        }
        else if (degrees < 0)
        {   // turn left.
            leftPower = power;

            robot.leftDriveBack.setPower(leftPower);
            robot.leftDriveFront.setPower(leftPower);

            robot.rightDriveBack.setPower(subpower);
            robot.rightDriveFront.setPower(subpower);
        }
        else return;

        // set power to rotate.



        while (true){
            int current = (int)this.getHeading();
            telemetry.addData("current", current);
            telemetry.addData("desired", getDesiredHeading());
            telemetry.update();
            if (!caller.opModeIsActive() || (degrees < 0 && current <= getDesiredHeading())
                    || (degrees > 0 && current >=  getDesiredHeading())){
                break;
            }
        }


        robot.leftDriveBack.setPower(0);
        robot.leftDriveFront.setPower(0);
        robot.rightDriveBack.setPower(0);
        robot.rightDriveFront.setPower(0);
    }

    public void pivot(int degrees, double power, double subPower, LinearOpMode caller){
//        correct();
        desiredHeading = degrees;
        double  leftPower = 0, rightPower = 0;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            rightPower = power;
            robot.rightDriveBack.setPower(rightPower);
            robot.rightDriveFront.setPower(rightPower);
            robot.leftDriveBack.setPower(subPower);
            robot.leftDriveFront.setPower(subPower);
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            robot.leftDriveBack.setPower(leftPower);
            robot.leftDriveFront.setPower(leftPower);
            robot.rightDriveBack.setPower(subPower);
            robot.rightDriveFront.setPower(subPower);
        }
        else return;

        // set power to rotate.



        while (true){
            int current = (int)this.getHeading();
            telemetry.addData("current", current);
            telemetry.addData("desired", getDesiredHeading());
            telemetry.update();
            if (!caller.opModeIsActive() || (degrees < 0 && current <= getDesiredHeading())
                    || (degrees > 0 && current >= getDesiredHeading())){
                break;
            }
        }


        robot.leftDriveBack.setPower(0);
        robot.leftDriveFront.setPower(0);
        robot.rightDriveBack.setPower(0);
        robot.rightDriveFront.setPower(0);
    }

    public void pivotBack(int degrees, double power){
//        correct();
        desiredHeading = degrees;
        double  leftPower = 0, rightPower = 0;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            rightPower = -power;
            robot.rightDriveBack.setPower(rightPower);
            robot.rightDriveFront.setPower(rightPower);
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            robot.leftDriveBack.setPower(leftPower);
            robot.leftDriveFront.setPower(leftPower);
        }
        else return;

        // set power to rotate.



        while (true){
            int current = (int)this.getHeading();
            telemetry.addData("current", current);
            telemetry.addData("desired", getDesiredHeading());
            telemetry.update();
            if (Math.abs(current) >= Math.abs((int) getDesiredHeading()))
            {
                break;
            }
        }


        robot.leftDriveBack.setPower(0);
        robot.leftDriveFront.setPower(0);
        robot.rightDriveBack.setPower(0);
        robot.rightDriveFront.setPower(0);
    }

    public void pivotBackReverse(int degrees, double power, LinearOpMode caller){
//        correct();
        desiredHeading = degrees;
        double  leftPower = 0, rightPower = 0;


        if (degrees < 0)
        {   // turn right.
            rightPower = -power;
            robot.rightDriveBack.setPower(rightPower);
            robot.rightDriveFront.setPower(rightPower);
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            robot.leftDriveBack.setPower(leftPower);
            robot.leftDriveFront.setPower(leftPower);
        }
        else return;

        // set power to rotate.



        while (true){
            int current = (int)this.getHeading();
            telemetry.addData("current", current);
            telemetry.addData("desired", getDesiredHeading());
            telemetry.update();
            if (!caller.opModeIsActive() || (Math.abs(current) <= Math.abs(getDesiredHeading())))
            {
                break;
            }
        }


        robot.leftDriveBack.setPower(0);
        robot.leftDriveFront.setPower(0);
        robot.rightDriveBack.setPower(0);
        robot.rightDriveFront.setPower(0);
    }

    public void pivotBackReverseAndWithdraw(int degrees, double power){
//        correct();
        desiredHeading = degrees;
        double  leftPower = 0, rightPower = 0;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            rightPower = -power;
            robot.rightDriveBack.setPower(rightPower);
            robot.rightDriveFront.setPower(rightPower);
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            robot.leftDriveBack.setPower(leftPower);
            robot.leftDriveFront.setPower(leftPower);
        }
        else return;

        // set power to rotate.
        robot.preMoveCrane(1, -10);

        boolean done = false;

        while (!done){
            if (!done) {
                int current = (int) this.getHeading();
                telemetry.addData("current", current);
                telemetry.addData("desired", getDesiredHeading());
                telemetry.update();
                if (Math.abs(current) <= Math.abs(getDesiredHeading())) {
                    done = true;
                    robot.stop();
                }
            }
        }

        robot.stop();
    }


    public void rotate(int degrees, double power)
    {
        double  leftPower = 0, rightPower = 0;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        robot.leftDriveBack.setPower(leftPower);
        robot.leftDriveFront.setPower(leftPower);
        robot.rightDriveBack.setPower(rightPower);
        robot.rightDriveFront.setPower(rightPower);

        while (true){
            int current = (int)getAngle();
            double correction = Math.abs(current) - Math.abs(globalAngle);
            if (correction >= MIN_TOLERANCE && correction <= MAX_TOLERANCE){
                break;
            }
        }


//        // rotate until turn is completed.
//        if (degrees < 0)
//        {
//            // On right turn we have to get off zero first.
////            while ( getAngle() == 0) {}
//
//            while ( getAngle() > degrees) {}
//        }
//        else    // left turn.
//            while (getAngle() < degrees) {}

        // turn the motors off.

        robot.leftDriveBack.setPower(0);
        robot.leftDriveFront.setPower(0);
        robot.rightDriveBack.setPower(0);
        robot.rightDriveFront.setPower(0);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public boolean isOffCourse(){
        int current = (int) getHeading();


        int absdesired = Math.abs(getDesiredHeading());
        int absCurrent = Math.abs(current);

        int correction = absCurrent - absdesired;
        if (correction >= MIN_TOLERANCE && correction <= MAX_TOLERANCE){
            return false;
        }
        else{
            return true;
        }
    }

    public void fixHeading(double power, LinearOpMode caller)
    {
        int current = (int) getHeading();
        double  leftPower = 0, rightPower = 0;

        int absdesired = Math.abs(getDesiredHeading());
        int absCurrent = Math.abs(current);

        int correction = absCurrent - absdesired;
        if (correction >= MIN_TOLERANCE && correction <= MAX_TOLERANCE){
            return;
        }

        if (current < getDesiredHeading()){
            //turn left
            leftPower = power;
            rightPower = -power;
        }else{
            //turn right
            leftPower = -power;
            rightPower = power;
        }



        // set power to rotate.
        robot.leftDriveBack.setPower(leftPower);
        robot.leftDriveFront.setPower(leftPower);
        robot.rightDriveBack.setPower(rightPower);
        robot.rightDriveFront.setPower(rightPower);

        while (true){
            current = (int)getHeading();
            correction = Math.abs(current) - absdesired;
            telemetry.addData("abscurrent", Math.abs(current));
            telemetry.addData("absdesired", absdesired);
            telemetry.addData("correction", correction);
            telemetry.update();

            if (!caller.opModeIsActive() || (correction >= MIN_TOLERANCE && correction <= MAX_TOLERANCE)){
                break;
            }
        }

        // turn the motors off.

        robot.leftDriveBack.setPower(0);
        robot.leftDriveFront.setPower(0);
        robot.rightDriveBack.setPower(0);
        robot.rightDriveFront.setPower(0);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void setTargetAngle(int angle){
        this.globalAngle = angle;
    }

    private void resetAngle()
    {
//        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

//        globalAngle = 0;
    }

    public int getDesiredHeading() {
        return desiredHeading;
    }

    public void setDesiredHeading(int desired) {
        desiredHeading = desired;
    }
}
