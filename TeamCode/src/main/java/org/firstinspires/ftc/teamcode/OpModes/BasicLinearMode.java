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

import org.firstinspires.ftc.teamcode.bots.TieBot;
import org.firstinspires.ftc.teamcode.skills.SoundEffect;
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

@TeleOp(name="Diagonal", group="Robot15173")
//@Disabled
public class BasicLinearMode extends LinearOpMode {

    // Declare OpMode members.
    TieBot robot   = new TieBot();
    private ElapsedTime     runtime = new ElapsedTime();
    StoneFinder finder;


    @Override
    public void runOpMode() {
        try {
            robot.init(this.hardwareMap, telemetry);
            SoundEffect soundEffect = new SoundEffect(this.hardwareMap);
            soundEffect.playTieFighter();
            finder = new StoneFinder(hardwareMap, telemetry);
            finder.initRec();
            waitForStart();
            runtime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                double drive = gamepad1.left_stick_x;
                if (drive < 0) {
                    robot.diagLeft(drive);
                }
                else if (drive > 0){
                    robot.diagRight(drive);
                }
                else{
                    robot.stop();
                }
                double dist = finder.getDistanceToObject();
                double angle = finder.getAngle();
                double left = finder.getStoneLeft();
                double center = finder.getStoneCenter();
                boolean detect = gamepad1.x;
                if (detect){
                    finder.detectStone(1, telemetry, this);
                    dist = finder.getDistanceToObject();
                    angle = finder.getAngle();
                    left = finder.getStoneLeft();
                    center = finder.getStoneCenter();
                }

                telemetry.addData("Left", left);
                telemetry.addData("Distance", dist);
                telemetry.addData("Angle", angle);
                telemetry.addData("Center", center);
                telemetry.update();

            }
        }
        catch (Exception ex){
            telemetry.addData("Issues with the OpMode", ex.getMessage());
            telemetry.update();
        }
        finally {
            if (finder != null){
                finder.stopStoneDetection();
            }
        }
    }
}
