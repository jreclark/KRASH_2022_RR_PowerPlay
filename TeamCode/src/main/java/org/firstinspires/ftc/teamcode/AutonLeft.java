package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AprilTags.AprilTagDetector;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Auto Left", group = "Comp", preselectTeleOp = "KRASH TeleOp")
//@Disabled
public class AutonLeft extends LinearOpMode {

    public Robot robot;
    public AprilTagDetector aprilTagDetector;
    public int sleeveVal = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, false);
        robot.arm.initGrabber();
        aprilTagDetector = new AprilTagDetector(hardwareMap, telemetry);
        aprilTagDetector.init();

        Pose2d startPose = new Pose2d(-31, -63, Math.toRadians(90));;

        robot.drive.setPoseEstimate(startPose);

        TrajectoryVelocityConstraint slowSpeed = robot.drive.getVelocityConstraint(25, MAX_ANG_VEL, TRACK_WIDTH);
        TrajectoryVelocityConstraint slowishSpeed = robot.drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH);

        TrajectorySequence firstDrop = robot.drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-13.5, -54.5), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-5, -29, Math.toRadians(45)), Math.toRadians(45))
                .build();

        TrajectorySequence firstPickup = robot.drive.trajectorySequenceBuilder(firstDrop.end())
                .setVelConstraint(slowishSpeed)
                .back(15)
                .setTangent(45)
                .splineToLinearHeading(new Pose2d(-24, -12, Math.toRadians(180)), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-62.5, -10))
                .build();

        TrajectorySequence secondDrop = robot.drive.trajectorySequenceBuilder(firstPickup.end())
                .setVelConstraint(slowishSpeed)
                .setTangent(0)
                .lineToSplineHeading(new Pose2d(-48, -10, Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(-5, -22, Math.toRadians(135)), Math.toRadians(-45))
                .build();

        TrajectorySequence parkRight = robot.drive.trajectorySequenceBuilder(secondDrop.end())
                .forward(15)
                .setTangent(-45)
                .splineToSplineHeading(new Pose2d(-12, -30, Math.toRadians(90)), Math.toRadians(-90))
                .build();

        TrajectorySequence parkMiddle = robot.drive.trajectorySequenceBuilder(secondDrop.end())
                .setTangent(135)
                .splineToSplineHeading(new Pose2d(-24, -12, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-38, -22, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        TrajectorySequence parkLeft = robot.drive.trajectorySequenceBuilder(secondDrop.end())
                .setVelConstraint(slowishSpeed)
                .setTangent(135)
                .splineToLinearHeading(new Pose2d(-24, -12, Math.toRadians(180)), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-63, -10))
                //.splineToSplineHeading(new Pose2d(-36, -12, Math.toRadians(180)), Math.toRadians(180))
                //.splineToSplineHeading(new Pose2d(-58, -22, Math.toRadians(-90)), Math.toRadians(-90))
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
            robot.drive.update();
        }

        robot.arm.setGrabberOpen();
        sleep(500);
        robot.arm.setRotateBack();
        sleep(1000);

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
        sleep(1000);

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
                grab = true;
                break;
            case 2:
                //  Insert code for Case 2
                park = parkMiddle;
                break;
            case 3:
                //  Insert code for Case 3
                park = parkRight;
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
            robot.arm.elevatorPositionControl(20);
        }



    }
}
