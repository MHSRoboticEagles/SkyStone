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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.TieBot;


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

@TeleOp(name="TeleOp", group="Robot15173")
@Disabled
public class SimpleLinearTurn extends LinearOpMode {

    // Declare OpMode members.
    TieBot robot   = new TieBot();
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        try {
            try {
                robot.init(this.hardwareMap, telemetry);
                robot.initStoneSensor();
            }
            catch (Exception ex){
                telemetry.addData("Init", ex.getMessage());
            }

            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {

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
                        robot.strafeRight(Math.abs(strafe));
                    } else {
                        robot.strafeLeft(Math.abs(strafe));
                    }
                } else {
                    robot.move(drive, turn);
                }

//                //diag
//                double diag = gamepad1.left_stick_x;
//                if (diag >= 0){
//                    robot.diagRight(diag, telemetry);
//                }
//                else{
//                    robot.diagLeft(-diag, telemetry);
//                }

                ///pivot
                boolean leftPivot = gamepad1.dpad_left;
                boolean rightPivot = gamepad1.dpad_right;
                if (leftPivot){
                    robot.pivotLeft(0.3);
                }
                else if(rightPivot){
                    robot.pivotRight(0.3);
                }

                boolean hook = gamepad1.x;
                boolean unhook = gamepad1.y;
                if (hook){
                    robot.hookTray(hook);
                }
                else if (unhook){
                    robot.hookTray(false);
                }


                //tower
                double tower = gamepad2.left_stick_y;
                robot.moveTower(tower);

                //crane
                double crane = gamepad2.right_stick_y;
                robot.moveCrane(-crane);

                //intake
                float pickup = gamepad2.right_trigger;
                if (pickup > 0) {
                    robot.moveIntake(pickup);
                }
                else {
                    //spit out
                    float spitOut = gamepad2.left_trigger;
                    robot.moveIntakeReverse(spitOut);
                }

                if (gamepad2.right_bumper){
                    robot.toggleStoneLock(true);
                }

                if (gamepad2.left_bumper){
                    robot.toggleStoneLock(false);
                }

                if (gamepad2.x){
                    robot.swivelStone(true);
                }
                else if (gamepad2.y) {
                    robot.swivelStone(false);
                }

                if (gamepad2.a){
                    robot.swivelStone90();
                }
                else if (gamepad2.y) {
                    robot.swivelStone(false);
                }


                if (gamepad1.a){
                    robot.positionCapstone();
                }

                if (gamepad1.b){
                    robot.resetCapstone();
                }

                robot.autoLockStone();

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
}
