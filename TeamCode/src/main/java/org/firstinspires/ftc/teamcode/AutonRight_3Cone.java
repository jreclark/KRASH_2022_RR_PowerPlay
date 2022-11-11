package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AprilTags.AprilTagDetector;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Auto Right- 3 Cone Old", group = "Comp", preselectTeleOp = "KRASH TeleOp")
@Disabled
public class AutonRight_3Cone extends LinearOpMode {

    public Robot robot;
    public AprilTagDetector aprilTagDetector;
    public int sleeveVal = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, false);
        robot.arm.initGrabber();
        aprilTagDetector = new AprilTagDetector(hardwareMap, telemetry);
        aprilTagDetector.init();

        Pose2d startPose = new Pose2d(31, -64, Math.toRadians(90));

        robot.drive.setPoseEstimate(startPose);

        TrajectoryVelocityConstraint slowSpeed = robot.drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH);

        TrajectorySequence firstDrop = robot.drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(14, -54), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(19.5, -7, Math.toRadians(45)), Math.toRadians(45))
                .build();

        TrajectorySequence firstPickup = robot.drive.trajectorySequenceBuilder(firstDrop.end())
                .setVelConstraint(slowSpeed)
                .setTangent(-135)
                .lineToLinearHeading(new Pose2d(8, -15, Math.toRadians(0)))
                .setTangent(0)
                .lineToLinearHeading(new Pose2d(62, -15, Math.toRadians(0)))
                .build();

        TrajectorySequence secondDrop = robot.drive.trajectorySequenceBuilder(firstPickup.end())
                .setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(45, -16, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(29, -6, Math.toRadians(-45)), Math.toRadians(135))
                .build();

        TrajectorySequence secondPickup = robot.drive.trajectorySequenceBuilder(secondDrop.end())
                .setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(62.5, -15, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence thirdDrop = robot.drive.trajectorySequenceBuilder(secondPickup.end())
                .setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(45, -16, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(29, -6, Math.toRadians(-45)), Math.toRadians(135))
                .build();

        TrajectorySequence parkRight = robot.drive.trajectorySequenceBuilder(thirdDrop.end())
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(62, -15, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence parkMiddle = robot.drive.trajectorySequenceBuilder(thirdDrop.end())
                .setTangent(Math.toRadians(-45))
                .splineToSplineHeading(new Pose2d(37, -20, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        TrajectorySequence parkLeft = robot.drive.trajectorySequenceBuilder(thirdDrop.end())
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

            robot.drive.update();
        }

        robot.arm.setGrabberOpen();
        sleep(500);
        robot.arm.setRotateBack();
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


        robot.drive.followTrajectorySequenceAsync(secondPickup);

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

        robot.arm.stackSecondGrab();

        robot.drive.followTrajectorySequenceAsync(thirdDrop);
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
                //grab = true;
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
            robot.drive.update();
        }

        robot.arm.elevatorPositionByConstant(Arm.ElevatorPositions.START_GROUND_GRAB);


    }
}
