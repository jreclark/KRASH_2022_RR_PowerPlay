package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AprilTags.AprilTagDetector;

@Autonomous(name = "Open Gripper", group = "Util")
//@Disabled
public class AutoOpenGripper extends LinearOpMode {

    public Robot robot;
    public AprilTagDetector aprilTagDetector;
    public int sleeveVal = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, false);
        robot.arm.setGrabberOpen();

        while(!isStarted()){}

    }
}
