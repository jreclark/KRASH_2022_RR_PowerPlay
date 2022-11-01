package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AprilTags.AprilTagDetector;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
//@Disabled
public class AutonTest extends LinearOpMode {

    public Robot robot;
    public AprilTagDetector aprilTagDetector;
    public int sleeveVal = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        robot.arm.initGrabber();
        //aprilTagDetector = new AprilTagDetector(hardwareMap, telemetry);
        //aprilTagDetector.init();

        Pose2d startPose = new Pose2d(34, -63, Math.toRadians(90));

        robot.drive.setPoseEstimate(startPose);

        TrajectorySequence firstDrop = robot.drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(36, -24, Math.toRadians(-90)), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    robot.arm.setRotateBack();
                })
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(24, 0, Math.toRadians(-45)), Math.toRadians(135))
                .build();

        TrajectorySequence firstPickup = robot.drive.trajectorySequenceBuilder(firstDrop.end())
                .splineToSplineHeading(new Pose2d(36, -24, Math.toRadians(-90)), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    robot.arm.setRotateBack();
                })
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(24, 0, Math.toRadians(-45)), Math.toRadians(135))
                .build();


        while (!isStarted() && !isStopRequested()){
            //sleeveVal = aprilTagDetector.updateAprilTagDetections();
            telemetry.addData("Position: ", sleeveVal);
            telemetry.update();
        }

        robot.arm.elevatorPositionByConstant(Arm.ElevatorPositions.HIGH);
        robot.drive.followTrajectorySequence(firstDrop);

        robot.arm.setGrabberOpen();
        sleep(500);


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
