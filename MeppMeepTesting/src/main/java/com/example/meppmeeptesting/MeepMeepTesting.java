package com.example.meppmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose = new Pose2d(40.5, -63, Math.toRadians(90));
        Pose2d startPose2 = new Pose2d(-31, -63, Math.toRadians(90));
        Pose2d startPose3 = new Pose2d(31, -64, Math.toRadians(90));
        ColorScheme scheme = new ColorSchemeBlueDark();

        RoadRunnerBotEntity compRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 14)
                .setDimensions(13, 14)
                .setColorScheme(scheme)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(31, -64, Math.toRadians(90)))
                                .setTangent(Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(14, -54), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(21, -3.5, Math.toRadians(45)), Math.toRadians(45))
                                //First Pickup
                                .setTangent(Math.toRadians(-45))
                                .lineToLinearHeading(new Pose2d(10, -13, Math.toRadians(0)))
                                .setTangent(Math.toRadians(0))
                                .lineToLinearHeading(new Pose2d(64.5, -12, Math.toRadians(0)))
                                //Second Drop
                                .setTangent(Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(45, -13, Math.toRadians(0)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(29, -3 , Math.toRadians(-45)), Math.toRadians(135))
                                //Second Pickup
                                .setTangent(Math.toRadians(-45))
                                .splineToLinearHeading(new Pose2d(64.5, -12, Math.toRadians(0)), Math.toRadians(0))
                                //Third Drop
                                .setTangent(Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(45, -13, Math.toRadians(0)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(29, -4, Math.toRadians(-45)), Math.toRadians(135))
                                //Park
                                .setTangent(Math.toRadians(-45))
                                .forward(4)
                                .setTangent(180)
                                .splineToSplineHeading(new Pose2d(12,-12, Math.toRadians(-90)), Math.toRadians(180))
                                .build()
                );

        RoadRunnerBotEntity compLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 14)
                .setDimensions(13, 14)
                .setColorScheme(scheme)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39, -64, Math.toRadians(90)))
                                .setTangent(Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(-13, -50), Math.toRadians(90))
                                //.splineToConstantHeading(new Vector2d(-15, -54), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(-20.5, -5, Math.toRadians(135)), Math.toRadians(135))
                                //First Pickup
                                .setTangent(Math.toRadians(-45))
                                .lineToLinearHeading(new Pose2d(-8, -13, Math.toRadians(180)))
                                .setTangent(Math.toRadians(0))
                                .lineToLinearHeading(new Pose2d(-63.5, -13, Math.toRadians(180)))
                                //Second Drop
                                .setTangent(Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-45, -14, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-29, -3 , Math.toRadians(-135)), Math.toRadians(45))
                                //Second Pickup
                                .setTangent(Math.toRadians(-135))
                                .splineToLinearHeading(new Pose2d(-63.5, -14, Math.toRadians(180)), Math.toRadians(180))
                                //Third Drop
                                .setTangent(Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-45, -15, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-30, -4.5, Math.toRadians(-135)), Math.toRadians(45))
                                //Park
                                .setTangent(Math.toRadians(-45))
                                .forward(9)
                                .setTangent(Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-14,-12, Math.toRadians(-90)), Math.toRadians(0))
                                .build()
                );

//        RoadRunnerBotEntity myBotRightAlt = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 14)
//                .setDimensions(14, 14)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(startPose3)
//                                .setTangent(Math.toRadians(180))
//                                .splineToConstantHeading(new Vector2d(13, -54), Math.toRadians(90))
//                                .splineToSplineHeading(new Pose2d(18, -4, Math.toRadians(45)), Math.toRadians(45))
//                                //Grab Second
//                                .back(12)
//                                .setTangent(0)
//                                .splineToSplineHeading(new Pose2d(63, -12, Math.toRadians(0)), Math.toRadians(0))
//                                .setTangent(Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(24, 0, Math.toRadians(-45)), Math.toRadians(135))
//                                .setTangent(Math.toRadians(-45))
//                                .splineToSplineHeading(new Pose2d(37, -20, Math.toRadians(-90)), Math.toRadians(-90))
//
//                                .build()
//                );
//
//        RoadRunnerBotEntity myBotLeft = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 14)
//                .setDimensions(14, 14)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(startPose2)
//                                .setTangent(Math.toRadians(0))
//                                .splineToConstantHeading(new Vector2d(-13, -54), Math.toRadians(90))
//                                .splineToSplineHeading(new Pose2d(-2, -26, Math.toRadians(45)), Math.toRadians(45))
//                                //Get the second
//                                .back(15)
//                                .setTangent(45)
//                                .splineToLinearHeading(new Pose2d(-24, -12, Math.toRadians(180)), Math.toRadians(180))
//                                .lineToConstantHeading(new Vector2d(-63, -12))
//                                //Drop Second
//                                .setTangent(Math.toRadians(180))
//                                .splineToSplineHeading(new Pose2d(45, -14, Math.toRadians(0)), Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(27.5, -5, Math.toRadians(-45)), Math.toRadians(135))
//                                .build()
//                );
//
//        RoadRunnerBotEntity right3Cone = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 14)
//                .setDimensions(14, 14)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(startPose3)
//                                .setTangent(Math.toRadians(180))
//                                .splineToConstantHeading(new Vector2d(14, -54), Math.toRadians(90))
//                                .splineToSplineHeading(new Pose2d(19.5, -7, Math.toRadians(45)), Math.toRadians(45))
//                                .setTangent(Math.toRadians(-135))
//                                .splineToConstantHeading(new Vector2d(10, -15), Math.toRadians(0))
//                                //.splineToConstantHeading(new Vector2d(8, -15), Math.toRadians(0))
////                                .lineToLinearHeading(new Pose2d(8, -15, Math.toRadians(0)))
//                                .setTangent(0)
//                                .splineToLinearHeading(new Pose2d(62, -15, Math.toRadians(0)), Math.toRadians(0))
//                                //Second Drop
//                                .setTangent(Math.toRadians(180))
//                                .splineToSplineHeading(new Pose2d(45, -16, Math.toRadians(0)), Math.toRadians(180))
//                                .splineToSplineHeading(new Pose2d(29, -6, Math.toRadians(-45)), Math.toRadians(135))
//                                //Second Pickup
//                                .setTangent(Math.toRadians(-45))
//                                .splineToLinearHeading(new Pose2d(62, -15, Math.toRadians(0)), Math.toRadians(0))
//                                //Third Drop
//                                .setTangent(Math.toRadians(180))
//                                .splineToSplineHeading(new Pose2d(45, -16, Math.toRadians(0)), Math.toRadians(180))
//                                .splineToSplineHeading(new Pose2d(29, -6, Math.toRadians(-45)), Math.toRadians(135))
//                                .build()
//                );




        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(compLeft)
                .start();
    }
}