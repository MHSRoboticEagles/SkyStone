package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.SimpleBot;


/**
 * Created by sjeltuhin on 1/15/18.
 */

public abstract class AutoBase extends LinearOpMode {

    protected SimpleBot robot = new SimpleBot();   // Use our standard robot configuration
    protected ElapsedTime runtime = new ElapsedTime();


    protected void runAutoMode(){
        initRobot();
        try {
            waitForStart();
            act();
        }
        catch (Exception ex){
            telemetry.addData("Issues autonomous initialization", ex);
            telemetry.update();
        }

    }

    protected void initRobot(){
        try{
        robot.init(hardwareMap, telemetry);
        }
        catch (Exception ex){
            telemetry.addData("Init", ex.getMessage());
        }
    }



    protected void act(){

    }


    protected void move(double speed, double moveTo){
        telemetry.addData("Auto", "Distance = %.2f", moveTo);
        telemetry.update();
        robot.encoderDrive(speed, moveTo, moveTo,0, telemetry);

        robot.stop();
    }

    protected void strafe(double speed, double moveTo){
        telemetry.addData("Auto", "Distance = %.2f", moveTo);
        telemetry.update();
        robot.encoderStrafe(speed, moveTo, 0, telemetry);
        robot.stop();
    }

    protected void turn(double speed, double moveTo){
        telemetry.addData("Auto", "Distance = %.2f", moveTo);
        telemetry.update();
        robot.encoderTurn(speed, moveTo, moveTo, 0, telemetry);
        robot.stop();
    }

    protected void unfoldIntake(double speed){
        robot.encoderIntakeUnfold(speed, 3, telemetry);
    }


}
