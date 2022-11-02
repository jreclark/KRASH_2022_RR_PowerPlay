package com.example.meppmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose = new Pose2d(40.5, -63, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 14)
                .setDimensions(14, 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .setTangent(Math.toRadians(145))
                                .splineToLinearHeading(new Pose2d(36, -20, Math.toRadians(-90)), Math.toRadians(90))
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(24, 0, Math.toRadians(-45)), Math.toRadians(135))
                                .setTangent(Math.toRadians(-45))
                                .splineToLinearHeading(new Pose2d(63, -12, Math.toRadians(0)), Math.toRadians(0))
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(24, 0, Math.toRadians(-45)), Math.toRadians(135))
                                .setTangent(Math.toRadians(-45))
                                .splineToLinearHeading(new Pose2d(63, -12, Math.toRadians(0)), Math.toRadians(0))
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(24, 0, Math.toRadians(-90)), Math.toRadians(90))
                                .setTangent(Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(12, -24, Math.toRadians(-90)),Math.toRadians(-90))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}