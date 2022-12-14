package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AprilTags.AprilTagDetector;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Auto Left - 3 Cones", group = "Comp", preselectTeleOp = "KRASH TeleOp")
//@Disabled
public class AutonLeft_3ConeWIP extends LinearOpMode {

    public Robot robot;
    public AprilTagDetector aprilTagDetector;
    public int sleeveVal = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, false);
        robot.arm.initGrabber();
        aprilTagDetector = new AprilTagDetector(hardwareMap, telemetry);
        aprilTagDetector.init();

        Pose2d startPose = new Pose2d(-39, -64, Math.toRadians(90));

        robot.drive.setPoseEstimate(startPose);

        TrajectoryAccelerationConstraint slowAccel = robot.drive.getAccelerationConstraint(20);
        TrajectoryAccelerationConstraint midAccel = robot.drive.getAccelerationConstraint(30);

        TrajectoryVelocityConstraint slowSpeed = robot.drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH);

        TrajectorySequence firstDrop = robot.drive.trajectorySequenceBuilder(startPose)
                .setAccelConstraint(slowAccel)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-13, -50), Math.toRadians(90))
                //.splineToConstantHeading(new Vector2d(-15, -54), Math.toRadians(90))
                .setAccelConstraint(midAccel)
                .splineToSplineHeading(new Pose2d(-22, -5.5, Math.toRadians(135)), Math.toRadians(135))
                .build();

        TrajectorySequence firstPickup = robot.drive.trajectorySequenceBuilder(firstDrop.end())
                //.setVelConstraint(slowSpeed)
                .setAccelConstraint(midAccel)
                .setTangent(Math.toRadians(-45))
                .lineToLinearHeading(new Pose2d(-8, -13, Math.toRadians(180)))
                .setTangent(Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(-64, -13, Math.toRadians(180)))
                .build();

        TrajectorySequence secondDrop = robot.drive.trajectorySequenceBuilder(firstPickup.end())
                //.setVelConstraint(slowSpeed)
                .setAccelConstraint(midAccel)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-45, -14, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-30, -4 , Math.toRadians(-135)), Math.toRadians(45))
                .build();

        TrajectorySequence secondPickup = robot.drive.trajectorySequenceBuilder(secondDrop.end())
                //.setVelConstraint(slowSpeed)
                .setAccelConstraint(midAccel)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(-64, -14, Math.toRadians(180)), Math.toRadians(180))
                .build();

        TrajectorySequence thirdDrop = robot.drive.trajectorySequenceBuilder(secondPickup.end())
                //.setVelConstraint(slowSpeed)
                .setAccelConstraint(slowAccel)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-45, -15, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-30, -4.5, Math.toRadians(-135)), Math.toRadians(45))
                .build();

        TrajectorySequence parkLeft = robot.drive.trajectorySequenceBuilder(thirdDrop.end())
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(-60, -15, Math.toRadians(180)), Math.toRadians(180))
                .build();

        TrajectorySequence parkMiddle = robot.drive.trajectorySequenceBuilder(thirdDrop.end())
                .setTangent(Math.toRadians(-135))
                .splineToSplineHeading(new Pose2d(-36, -20, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        TrajectorySequence parkRight = robot.drive.trajectorySequenceBuilder(thirdDrop.end())
                .setTangent(Math.toRadians(135))
                .forward(9)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-14,-12, Math.toRadians(-90)), Math.toRadians(0))
                //.forward(12)
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

        if(isStopRequested()) return;

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
        while (robot.drive.isBusy() && !isStopRequested()) {
            if (robot.arm.isSafeToRotate()) {
                robot.arm.setRotateFront();
            }
            robot.drive.update();
        }

        robot.drive.setMotorPowers(0, 0, 0, 0);
        robot.arm.elevatorPositionByConstant(Arm.ElevatorPositions.START_GROUND_GRAB);


    }
}
