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

@TeleOp(name="KRASH TeleOp", group="Comp")
//@Disabled
public class BasicDriving extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();
    private Robot myRobot;
    private double scaleFactor = 1;

    private boolean slowMode = false;
    private double SLOW_SCALE_FACTOR = 0.5;
    private double driveScaleFactor = 1;

    private boolean elevatorInHold = true;
    private boolean elevatorManualOp = false;

    @Override
    public void runOpMode() {
        myRobot = new Robot(hardwareMap, telemetry);


        double drive;
        double turn;
        double strafe;
        double leftFrontPower, leftRearPower, rightFrontPower, rightRearPower;

        double armPower;

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Press start to begin ...");
        telemetry.update();


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        // should ALWAYS be checking opModeIsActive() in teleOp loop
        while (opModeIsActive()) {

            /***************************************************************
             * Driver Controls
             ***************************************************************/
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;

            if(gamepad1.left_bumper){
                driveScaleFactor = SLOW_SCALE_FACTOR;
            } else if(gamepad1.right_bumper){
                driveScaleFactor = 1;
            }

            myRobot.drive.teleDrive(drive, turn, strafe, driveScaleFactor);

            /***************************************************************
             * Manipulator Controls
             ***************************************************************/
            //Elevator Control
            armPower = -gamepad2.right_stick_y;

//            myRobot.arm.runElevator(armPower);
            if (Math.abs(armPower) > 0.05) {
                myRobot.arm.runElevator(armPower);
                elevatorInHold = false;
                elevatorManualOp = true;
            } else if (elevatorManualOp && !elevatorInHold) {
                myRobot.arm.holdElevator();
                elevatorInHold = true;
                elevatorManualOp = false;
            }

            if (gamepad2.dpad_up) {
                myRobot.arm.elevatorPositionByConstant(Arm.ElevatorPositions.HIGH);
                if(myRobot.arm.isSafeToRotate()){
                    myRobot.arm.setRotateBack();
                }
                elevatorManualOp = false;
            } else if (gamepad2.dpad_right) {
                myRobot.arm.elevatorPositionByConstant(Arm.ElevatorPositions.MID);
                if(myRobot.arm.isSafeToRotate()){
                    myRobot.arm.setRotateBack();
                }
                elevatorManualOp = false;
            } else if (gamepad2.dpad_down) {
                myRobot.arm.elevatorPositionByConstant(Arm.ElevatorPositions.SHORT);
                if(myRobot.arm.isSafeToRotate()){
                    myRobot.arm.setRotateBack();
                }
                elevatorManualOp = false;
            } else if (gamepad2.dpad_left){
                myRobot.arm.elevatorPositionByConstant(Arm.ElevatorPositions.LOW);
            } else if (gamepad2.a) {
                if(myRobot.arm.isRotateFront()){
                    myRobot.arm.elevatorPositionByConstant(Arm.ElevatorPositions.START_GROUND_GRAB);
                } else if(myRobot.arm.isSafeToRotate()){
                    myRobot.arm.setRotateFront();
                } else {
                    myRobot.arm.elevatorPositionByConstant(Arm.ElevatorPositions.PIVOT_POINT);
                }
            }

            if(gamepad2.right_trigger>0.85 && myRobot.arm.isRotateFront()){
                myRobot.arm.groundGrab();
            } else if(gamepad2.right_trigger > 0.1){
                myRobot.arm.setGrabberClosed();
            } else if(gamepad2.left_trigger > 0.1){
                myRobot.arm.setGrabberOpen();
            }

            if(gamepad2.right_bumper && myRobot.arm.isSafeToRotate()){
                myRobot.arm.setRotateFront();
            } else if(gamepad2.left_bumper && myRobot.arm.isSafeToRotate()){
                myRobot.arm.setRotateBack();
            }

            //Update PIDF Values.
            //TODO: Can delete once tuning is complete.
//            if(gamepad2.a){
//                myRobot.arm.setPIDFValues();
//            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Elevator Position", myRobot.arm.getElevatorPosition());
            telemetry.addData("Elevator in hold", elevatorInHold);
            telemetry.addData("Elevator busy", myRobot.arm.elevatorIsBusy());
            //myRobot.arm.telemetryPIDF();
            telemetry.update();

        }

    }

}
