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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Utils;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic Drive", group="Linear Opmode")
//@Disabled
public class BasicDriving extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Robot myRobot;
    private double scaleFactor = 1;

    private boolean slowMode = false;
    private double SLOW_SCALE_FACTOR = 0.5;
    private double driveScaleFactor = 1;

    @Override
    public void runOpMode() {
        myRobot = new Robot(hardwareMap, telemetry);

        double drive;
        double turn;
        double strafe;
        double leftFrontPower, leftRearPower, rightFrontPower, rightRearPower;

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Press start to begin ...");
        telemetry.update();


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        // should ALWAYS be checking opModeIsActive() in teleOp loop
        while (opModeIsActive()) {

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;

            if(gamepad1.left_bumper){
                driveScaleFactor = SLOW_SCALE_FACTOR;
            } else if(gamepad1.right_bumper){
                driveScaleFactor = 1;
            }

            leftFrontPower    = drive + turn + strafe;
            leftRearPower   = drive + turn - strafe;
            rightFrontPower    = drive - turn - strafe;
            rightRearPower   = drive - turn + strafe;

            scaleFactor = driveScaleFactor * Utils.scalePower(leftFrontPower, leftRearPower, rightFrontPower, rightRearPower);

            leftRearPower = leftRearPower * scaleFactor;
            leftFrontPower = leftFrontPower * scaleFactor;
            rightFrontPower = rightFrontPower * scaleFactor;
            rightRearPower = rightRearPower * scaleFactor;

            // Send calculated power to wheels
            myRobot.drive.setMotorPowers(leftFrontPower, leftRearPower, rightFrontPower, rightRearPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "leftFront (%.2f), rightFront (%.2f)", leftFrontPower, rightFrontPower);
            telemetry.update();

        }

    }

}
