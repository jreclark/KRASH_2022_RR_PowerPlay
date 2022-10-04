package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.AprilTags.AprilTagDetector;
import org.firstinspires.ftc.teamcode.drive.DriveByEncoders;

@Autonomous
@Disabled
public class AutonWithVisionFramework extends LinearOpMode {

    public Robot robot;
    public AprilTagDetector aprilTagDetector;
    public int sleeveVal = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        aprilTagDetector = new AprilTagDetector(hardwareMap, telemetry);



        while (!isStarted() && !isStopRequested()){
            sleeveVal = aprilTagDetector.updateAprilTagDetections();
        }

        switch (sleeveVal){
            case 0:
                // Insert code for when no tag is detected.  If you want this to default to one of the other cases,
                // put this case block before that case and leave it blank.  For example, want to run case #1,
                // leave this block of code where it is.
            case 1:
                //  Insert code for Case 1
                break;
            case 2:
                //  Insert code for Case 2
                break;
            case 3:
                //  Insert code for Case 3
                break;
            default:
                // Insert code here to capture any other values.  Should be impossible base on how updateAprilTagDetections() was originally written.
                break;
        }



    }
}
