package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        System.setProperty("sun.java2d.opengl", "true");

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.PI, Math.PI, 10.803180)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(64.75, -7.125, Math.toRadians(180)))
                .strafeToSplineHeading(new Vector2d(60, -7.125), Math.toRadians(200))
                .waitSeconds(5)
                .strafeToSplineHeading(new Vector2d(37, -29), Math.toRadians(269))
                .strafeTo(new Vector2d(37, -52))
                .strafeToSplineHeading(new Vector2d(60, -7.125), Math.toRadians(200))
                .waitSeconds(3)
                .strafeToSplineHeading(new Vector2d(13, -29), Math.toRadians(269))
                .strafeTo(new Vector2d(13, -52))
                .strafeToSplineHeading(new Vector2d(60, -7.125), Math.toRadians(200))
                .waitSeconds(3)
                .strafeToSplineHeading(new Vector2d(2, -58), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-10.5, -29), Math.toRadians(269))
                .strafeTo(new Vector2d(-10.5, -52))
                .strafeToLinearHeading(new Vector2d(-10, -10), Math.toRadians(225))
                .waitSeconds(3)
                .build());

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(64.75, 7.125, Math.toRadians(180)))
                .strafeToSplineHeading(new Vector2d(60, 7.125), Math.toRadians(160))
                .waitSeconds(5)
                .strafeToSplineHeading(new Vector2d(37, 29), Math.toRadians(90))
                .strafeTo(new Vector2d(37, 52))
                .strafeToSplineHeading(new Vector2d(60, 7.125), Math.toRadians(160))
                .waitSeconds(3)
                .strafeToSplineHeading(new Vector2d(13, 29), Math.toRadians(90))
                .strafeTo(new Vector2d(13, 52))
                .strafeToSplineHeading(new Vector2d(60, 7.125), Math.toRadians(160))
                .waitSeconds(3)
                .strafeToSplineHeading(new Vector2d(2, 58), Math.toRadians(270))
                .strafeToSplineHeading(new Vector2d(-10.5, 29), Math.toRadians(90))
                .strafeTo(new Vector2d(-10.5, 52))
                .strafeToLinearHeading(new Vector2d(-10, 10), Math.toRadians(135))
                .waitSeconds(3)
                .build());

//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
//                .lineToX(30)
//                .turn(Math.toRadians(90))
//                .lineToY(30)
//                .turn(Math.toRadians(90))
//                .lineToX(0)
//                .turn(Math.toRadians(90))
//                .lineToY(0)
//                .turn(Math.toRadians(90))
//                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}