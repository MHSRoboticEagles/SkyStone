/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.bots.SimpleBot;
import org.firstinspires.ftc.teamcode.skills.DetectionInterface;
import org.firstinspires.ftc.teamcode.skills.StoneFinder;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="SkyStoneDetection", group="Robot15173")
//@Disabled
public class SkyStoneDetection extends LinearOpMode {

    // Declare OpMode members.
    SimpleBot robot   = new SimpleBot();
    private ElapsedTime     runtime = new ElapsedTime();
    private StoneFinder stoneFinder = null;
    private float stoneLeft = 0;

    private VuforiaLocalizer vuforia;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY =
            "AZs0syj/////AAABmaxIME6H4k74lx12Yv3gnoYvtGHACOflWi3Ej36sE7Hn86xDafDA3vhzxSOgBtyNIQ1ua6KP2j3I2ScFedVw8n6MJ7PReZQP4sTdc8gHvoy17hD574exMmoUQ3rVUMkgU2fwN2enw2X+Ls2F3BLuCg/A4SBZjzG3cweO+owiKO/2iSIpeC4rBdUnPiTxqPHNa8UOxyncCGV0+ZFXresQm/rK7HbOKB9MEszi8eW2JNyfjTdKozwDxikeDRV7yPvoIhZ5A+mrrC1GgrEzYwNTVHeki2cg4Ea62pYwdscaJ+6IWHlBIDutqmgJu/Os3kAzZkOh0TJ8P3e29Ou4ZczTdDH0oqkPt78Nt4VdYbSbLRCw";


    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    @Override
    public void runOpMode() {
        try {
            try {
                robot.init(this.hardwareMap, telemetry);

                initVuforia();
                if (vuforia == null){
                    telemetry.addData("Error", "Unable to start Vuforia");
                }
                else {
                    telemetry.addData("Info", "Vuforia initialized");

                    try {

                        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                            initTfod();
                            telemetry.addData("Info", "TF initialized");
                        } else {
                            telemetry.addData("Error", "This device is not compatible with TFOD");
                        }

                        /**
                         * Activate TensorFlow Object Detection before we wait for the start command.
                         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
                         **/
                        if (tfod != null) {
                            tfod.activate();
                        }
                        telemetry.addData("Info", "TF Activated");
                    }
                    catch (Exception ex){
                        telemetry.addData("Error", "Unable to initialize Tensor Flow");
                    }
                }

//                stoneFinder = new StoneFinder(new DetectionInterface() {
//                    @Override
//                    public void stoneFound(float left) {
//                        stoneLeft = left;
//                        stoneFinder.stop();
//                    }
//                }, tfod);

            }
            catch (Exception ex){
                telemetry.addData("Init", ex.getMessage());
            }

            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();
//            stoneFinder.run();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                telemetry.addData("Main loop", "Running");
                telemetry.addData("Stone Left", this.stoneLeft);
                // POV Mode uses left stick to go forward, and right stick to turn.
                // - This uses basic math to combine motions and is easier to drive straight.
                double drive = gamepad1.left_stick_y;
                double turn = 0;
                double ltrigger = gamepad1.left_trigger;
                double rtrigger = gamepad1.right_trigger;
                if (ltrigger > 0){
                    turn = -ltrigger;
                }
                else if (rtrigger > 0){
                    turn = rtrigger;
                }

                double strafe = gamepad1.right_stick_x;


                if (Math.abs(strafe) > 0) {
                    telemetry.addData("Strafing", "Left: %2f", strafe);
                    telemetry.update();
                    if (strafe < 0) {
                        robot.strafeRight(Math.abs(strafe), telemetry);
                    } else {
                        robot.strafeLeft(Math.abs(strafe), telemetry);
                    }
                } else {
                    robot.move(drive, turn, telemetry);
                }

                ///pivot
                boolean leftPivot = gamepad1.dpad_left;
                boolean rightPivot = gamepad1.dpad_right;
                if (leftPivot){
                    robot.pivotLeft(1, telemetry);
                }
                else if(rightPivot){
                    robot.pivotRight(1, telemetry);
                }

                //platform
                double plat = gamepad2.left_stick_x;
                robot.movePlatform(plat, telemetry);


                //tower
                double tower = gamepad2.left_stick_y;
                robot.moveTower(tower, telemetry);

                float pickup = gamepad2.right_trigger;
                if (pickup > 0) {
                    telemetry.addData("Intake pickup", "Speed from %.2f", pickup);
                    robot.pickupTemp(pickup, telemetry);
                }

                float dump = gamepad2.left_trigger;
                if (dump > 0) {
                    telemetry.addData("Intake dump", "Speed from %.2f", dump);
                    robot.releaseTemp(dump, telemetry);
                }

                //intakePivot
                double intakePivot = gamepad2.right_stick_y;
                robot.moveIntake(intakePivot, telemetry);

                robot.getGyro().getOrientation();
//                robot.getGyro().getPosition();

                if (gamepad1.x){
                    robot.getGyro().correct();
                }


                telemetry.update();
            }
        }
        catch (Exception ex){
            telemetry.addData("Issues with the OpMode", ex.getMessage());
            telemetry.update();
        }
        finally {
            if (robot != null){
//                robot.getGyro().stopRecordingAcceleration();
            }
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}
