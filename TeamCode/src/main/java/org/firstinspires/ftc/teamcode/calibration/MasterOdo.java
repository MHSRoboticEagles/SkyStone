package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.bots.YellowBot;

import java.io.File;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Master Odo", group="Robot15173")
public class MasterOdo extends LinearOpMode {

    protected YellowBot bot = new YellowBot();
    ElapsedTime timer = new ElapsedTime();

    private static String CONFIG_FILE_NAME = "master-odo-config.json";

    protected double right = 0;
    protected double left = 0;
    protected double valueHead = 0;
    protected double startHead = 0;
    protected double nextHead = 0;
    protected double desiredHead = 0;
    protected double leftTarget = 0;
    protected double rightTarget = 0;

    protected double ratio = 0;

    protected double startX = 0;
    protected double startY = 0;

    protected double targetX = 0;
    protected double targetY = 0;

    protected double valueX = 0;
    protected double valueY = 0;

    private boolean xmode = false;
    private boolean ymode = false;
    private boolean speedMode = false;
    private boolean headingMode = false;

    private double MODE_VALUE = 1;
    private boolean MODE_UP = true;



    private boolean MOVING = false;


    private double DISTANCE = 0;

    private double SPEED = bot.CALIB_SPEED;
    protected double valueSpeed = SPEED;
    private double SPEED_INCREMENT = 0.1;

    private double COORD_MIN = 1;
    private double COORD_MAX = 100;
    private double COORD_MULTIPLIER = 10;

    Deadline gamepadRateLimit;
    private final static int GAMEPAD_LOCKOUT = 500;


    @Override
    public void runOpMode() throws InterruptedException {
        try {
            bot.init(this, hardwareMap, telemetry);
            bot.initGyro();
            bot.initCalibData();
            gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);

            restoreConfig();
//            showSettings();

            waitForStart();
            showSettings();

            while (opModeIsActive()) {

                processCommands();

            }
        }
        catch (Exception ex){
            telemetry.addData("Error", ex.getMessage());
            telemetry.update();
        }
    }

    private void processCommands(){
        if (!gamepadRateLimit.hasExpired()) {
            return;
        }
        if (gamepad1.x){
            MOVING = false;
            if (xmode == false){
                xmode = true;
                showConfig();
            }
            else{
                //accept x value
                xmode = false;
                targetX = valueX;
                showSettings();
            }
            gamepadRateLimit.reset();
        }

        if (gamepad1.y){
            MOVING = false;
            if (ymode == false){
                ymode = true;
                showConfig();
            }
            else{
                //accept y value
                ymode = false;
                targetY = valueY;
                showSettings();
            }
            gamepadRateLimit.reset();
        }

        if (gamepad1.a){
            MOVING = false;
            if (speedMode == false){
                speedMode = true;
                showConfig();
            }
            else{
                speedMode = false;
                SPEED = valueSpeed;
                showSettings();
            }
            gamepadRateLimit.reset();
        }

        if (gamepad1.b){
            MOVING = false;
            if (headingMode == false){
                headingMode = true;
                showConfig();
            }
            else{
                headingMode = false;
                desiredHead = valueHead;
                showSettings();
            }
            gamepadRateLimit.reset();
        }

        if (gamepad1.back){
            MOVING = false;
            if (xmode){
                valueX = 0;
            }

            if (ymode){
                valueY = 0;
            }
            if (speedMode){
                valueSpeed = 0;
            }

            if (headingMode){
                valueHead = 0;
            }
            showSettings();
            gamepadRateLimit.reset();
        }

        if (gamepad1.dpad_left ){
            MOVING = false;
            MODE_VALUE = -changeIncrement();
            showConfig();
            gamepadRateLimit.reset();
        }

        if (gamepad1.dpad_right){
            MOVING = false;

            MODE_VALUE = changeIncrement();
            showConfig();
            gamepadRateLimit.reset();
        }

        //value adjustment
        if (gamepad1.dpad_up){
            MOVING = false;
            if (xmode){
                valueX = valueX + MODE_VALUE;
            }

            if (ymode){
                valueY = valueY + MODE_VALUE;
            }

            if (speedMode && valueSpeed < 1){
                valueSpeed = valueSpeed + SPEED_INCREMENT;
            }

            if (headingMode && valueHead < 180){
                valueHead = valueHead + Math.abs(MODE_VALUE);
            }
            showConfig();
            gamepadRateLimit.reset();
        }

        if (gamepad1.dpad_down){
            MOVING = false;
            if (xmode){
                valueX -= MODE_VALUE;
            }

            if (ymode){
                valueY -= MODE_VALUE;
            }

            if (speedMode && valueSpeed > 0){
                valueSpeed -= SPEED_INCREMENT;
            }

            if (headingMode && valueHead > -180){
                valueHead = valueHead - Math.abs(MODE_VALUE);
            }

            showConfig();
            gamepadRateLimit.reset();
        }



        if (!MOVING&& gamepad1.right_bumper){
            gamepadRateLimit.reset();
            saveConfig();

            MOVING = true;
            if (startX != targetX || startY != targetY) {
                DISTANCE = targetX;
                ratio = moveForward();
            }
            else{
                spin();
            }

            showStats();
        }

    }

    private double changeIncrement(){
        double tempVal = Math.abs(MODE_VALUE);
        if(MODE_UP && tempVal < COORD_MAX){
            tempVal = tempVal * COORD_MULTIPLIER;
            if (tempVal == COORD_MAX) {
                MODE_UP = false;
            }
        }
        else {
            if (MODE_UP == false && tempVal > COORD_MIN) {
                tempVal = tempVal / COORD_MULTIPLIER;
                if (tempVal == COORD_MIN) {
                    MODE_UP = true;
                }
            }
        }
        return tempVal;
    }

    private double moveForward(){
        startHead = bot.getGyroHeading();
        double leftStart = bot.getLeftOdemeter();
        double rightStart = bot.getRightOdemeter();
        leftTarget = bot.getLeftTarget(DISTANCE);
        rightTarget = bot.getRightTarget(DISTANCE);
        bot.moveTo(SPEED, SPEED, DISTANCE);
        this.left = bot.getLeftOdemeter() - leftStart;
        this.right = bot.getRightOdemeter() - rightStart;

        double ratio = right/left;

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("Calib","Waiting for gyro to settle ...");
            telemetry.update();
        }
        nextHead = bot.getGyroHeading();

        return ratio;
    }

    private void spin(){
        startHead = bot.getGyroHeading();

        bot.spin(desiredHead, SPEED);

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("Gyroscope","Stabilizing ...");
            telemetry.update();
        }

        nextHead = bot.getGyroHeading();
    }

    private void showSettings(){
        telemetry.addData("Target X", "%.2f. To change press x", targetX);
        telemetry.addData("Target Y", "%.2f. To change press y", targetY);
        telemetry.addData("Heading", "%.2f. To change press b", desiredHead);
        telemetry.addData("Speed", "%.2f. To change press a", SPEED);
        telemetry.addData("Start", "Press right bumper");
        telemetry.update();
    }

    private void showConfig(){
        if (xmode){
            telemetry.addData("Config", "Setting x");
            telemetry.addData("X", valueX);
            telemetry.addData("Mode", MODE_VALUE);
            showHints(true);
            telemetry.addData("To save", "Press X");
        }
        if (ymode){
            telemetry.addData("Config", "Setting y");
            telemetry.addData("Y", valueY);
            telemetry.addData("Mode", MODE_VALUE);
            showHints(true);
            telemetry.addData("To save", "Press Y");
        }
        if (speedMode){
            telemetry.addData("Config", "Setting speed");
            telemetry.addData("Speed", "%.2f", valueSpeed);
            showHints(false);
            telemetry.addData("To save", "Press A");
        }

        if (headingMode){
            telemetry.addData("Config", "Setting heading");
            telemetry.addData("Heading", "%.2f", valueHead);
            telemetry.addData("Mode", MODE_VALUE);
            showHints(false);
            telemetry.addData("To save", "Press B");
        }

        telemetry.update();
    }

    private void showHints(boolean full){
        telemetry.addData("DPAD UP", "Increment value");
        telemetry.addData("DPAD DOWN", "Decrease value");
        if (full) {
            telemetry.addData("DPAD Left/Right:", "Change increment: -100 -10 -1 1 10 100");
        }
    }


    private void showHelp(){
        telemetry.addData("X:", "Set the x coordinate value");
        telemetry.addData("Y", "Set the y coordinate value");
        telemetry.addData("A", "Set speed in increments of 0.1");
        telemetry.addData("Right Bumper", "Set the robot in motion");
        telemetry.update();
    }

    private void showStats(){
        telemetry.addData("Moving to ", "%.2f : %.2f", targetX, targetY);
        telemetry.addData("ratio", ratio);
        telemetry.addData("startHead", startHead);
        telemetry.addData("nextHead", nextHead);
        telemetry.addData("Speed", SPEED);

        telemetry.update();
    }

    private void saveConfig(){
        try{
            LastRunConfig config = new LastRunConfig();
            config.setX(targetX);
            config.setY(targetY);
            config.setSpeed(SPEED);
            config.setHeading(desiredHead);
            File configFile = AppUtil.getInstance().getSettingsFile(CONFIG_FILE_NAME);
            ReadWriteFile.writeFile(configFile, config.serialize());
        }
        catch (Exception e) {
            telemetry.addData("Error", "Config cannot be saved. %s", e.getMessage());
            telemetry.update();
        }
    }


    private void restoreConfig(){
        try{
            File configFile = AppUtil.getInstance().getSettingsFile(CONFIG_FILE_NAME);
            if (configFile.exists()) {
                String data = ReadWriteFile.readFile(configFile);
                LastRunConfig config = LastRunConfig.deserialize(data);
                if (config != null){
                    targetX = config.getX();
                    targetY = config.getY();
                    SPEED = config.getSpeed();
                    desiredHead = config.getHeading();
                    valueX = targetX;
                    valueY = targetY;
                    valueSpeed = SPEED;
                    valueHead = desiredHead;
                }
                else{
                    telemetry.addData("Info", "Config file empty or could not be read");
                    telemetry.update();
                }
            }
            else{
                telemetry.addData("Info", "Config does not exist");
                telemetry.update();
            }
        }
        catch (Exception e) {
            telemetry.addData("Error", "Issues when reading file. %s", e.getMessage());
            telemetry.update();
        }
    }


}
