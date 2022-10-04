package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveByEncoders;

@Autonomous
@Disabled
public class Auton_DriveByEncoders extends LinearOpMode {

    public Robot robot;
    public DriveByEncoders autoDrive;

    private double gyroTurnTimeout = 1;
    private double gyroHoldTimeout = 1.0;

    double gyroTurnPowr = 0.7;
    double gyroHoldPowr = 0.6;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        autoDrive = new DriveByEncoders(hardwareMap, robot.drive, telemetry);

        autoDrive.InitializeGryo();

        //TODO: Add vision handling.  Should result in markerLocation indicating marker position.
        while (!isStarted()){

        }


        while(opModeIsActive()){

            autoDrive.DriveForward(24, 0.5, 3);
            autoDrive.StrafeRight(12, 0.5, 2);

            autoDrive.gyroTurn(0.8, +45.0, 0.5);
            autoDrive.gyroHold(0.3, +45.0, 1.5);



            telemetry.addData("Left Front", robot.drive.leftFront.getCurrentPosition());
            telemetry.update();
        }

    }
}
