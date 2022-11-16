package org.firstinspires.ftc.teamcode.ARCHIVE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AprilTags.AprilTagDetector;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Auto Right", group = "Comp", preselectTeleOp = "BasicDriving")
@Disabled
public class AutonRight extends LinearOpMode {

    public Robot robot;
    public AprilTagDetector aprilTagDetector;
    public int sleeveVal = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, false);
        robot.arm.initGrabber();
        aprilTagDetector = new AprilTagDetector(hardwareMap, telemetry);
        aprilTagDetector.init();

        Pose2d startPose = new Pose2d(40, -64, Math.toRadians(90));

        robot.drive.setPoseEstimate(startPose);

        TrajectorySequence firstDrop = robot.drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(145.0))
                .splineToSplineHeading(new Pose2d(36, -20, Math.toRadians(-90)), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(28.5, -4.5, Math.toRadians(-45)), Math.toRadians(135))
                .build();

        TrajectorySequence firstPickup = robot.drive.trajectorySequenceBuilder(firstDrop.end())
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(62, -14, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence secondDrop = robot.drive.trajectorySequenceBuilder(firstPickup.end())
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(28.5, -4.5, Math.toRadians(-45)), Math.toRadians(135))
                .build();

        TrajectorySequence parkRight = robot.drive.trajectorySequenceBuilder(secondDrop.end())
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(62, -14, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence parkMiddle = robot.drive.trajectorySequenceBuilder(secondDrop.end())
                .setTangent(Math.toRadians(-45))
                .splineToSplineHeading(new Pose2d(37, -20, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        TrajectorySequence parkLeft = robot.drive.trajectorySequenceBuilder(secondDrop.end())
                .setTangent(-45)
                .forward(8)
                .setTangent(180)
                .splineToSplineHeading(new Pose2d(12,-12, Math.toRadians(-90)), Math.toRadians(180))
                .forward(12)
                .build();

        TrajectorySequence backup = robot.drive.trajectorySequenceBuilder(secondDrop.end())
                .setTangent(Math.toRadians(-45))
                .back(12)
                .build();


        while (!isStarted() && !isStopRequested()){
            sleeveVal = aprilTagDetector.updateAprilTagDetections();
            telemetry.addData("Position: ", sleeveVal);
            telemetry.update();
        }

        robot.arm.elevatorPositionByConstant(Arm.ElevatorPositions.HIGH);
        robot.drive.followTrajectorySequenceAsync(firstDrop);
        while(robot.drive.isBusy()){
            if(robot.arm.isSafeToRotate()){
                robot.arm.setRotateBack();
            }
            robot.drive.update();
        }

        robot.arm.setGrabberOpen();
        sleep(500);

        robot.drive.followTrajectorySequenceAsync(firstPickup);

        while (robot.drive.isBusy()){
            if(robot.arm.isRotateFront()){
                robot.arm.elevatorPositionByConstant(Arm.ElevatorPositions.START_AUTO_GRAB);
            } else if(robot.arm.isSafeToRotate()){
                robot.arm.setRotateFront();
            } else {
                robot.arm.elevatorPositionByConstant(Arm.ElevatorPositions.PIVOT_POINT);
            }
            robot.drive.update();
        }

        robot.arm.stackFirstGrab();

        robot.drive.followTrajectorySequenceAsync(secondDrop);
        while(robot.drive.isBusy()){
            if(robot.arm.isSafeToRotate()){
                robot.arm.setRotateBack();
            }
            robot.drive.update();
        }

        robot.arm.setGrabberOpen();
        sleep(500);

        TrajectorySequence park;
        boolean grab = false;

        switch (sleeveVal){
            case 0:
                // Insert code for when no tag is detected.  If you want this to default to one of the other cases,
                // put this case block before that case and leave it blank.  For example, want to run case #1,
                // leave this block of code where it is.
                park = parkMiddle;
                break;
            case 1:
                //  Insert code for Case 1
                park = parkLeft;
                break;
            case 2:
                //  Insert code for Case 2
                park = parkMiddle;
                break;
            case 3:
                //  Insert code for Case 3
                park = parkRight;
                grab = true;
                break;
            default:
                // Insert code here to capture any other values.  Should be impossible base on how updateAprilTagDetections() was originally written.
                park = backup;
                break;
        }

        robot.drive.followTrajectorySequenceAsync(park);
        while (robot.drive.isBusy()) {
            if (robot.arm.isSafeToRotate()) {
                robot.arm.setRotateFront();
            }
            if (robot.arm.isRotateFront()) {
                if (grab) {
                    robot.arm.elevatorPositionByConstant(Arm.ElevatorPositions.START_AUTO_GRAB);
                } else {
                    robot.arm.elevatorPositionByConstant(Arm.ElevatorPositions.START_GROUND_GRAB);
                }
            }
            robot.drive.update();
        }

        if (grab){
            robot.arm.stackSecondGrabbyandHold();
            robot.arm.setRotateBack();
            sleep(1000);
            robot.arm.elevatorPositionByConstant(Arm.ElevatorPositions.LOW);
        }





    }
}
