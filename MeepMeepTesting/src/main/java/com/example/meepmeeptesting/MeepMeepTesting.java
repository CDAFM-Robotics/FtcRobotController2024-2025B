package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting{
  public static void main(String[] args) {
    MeepMeep meepMeep = new MeepMeep(640);

    RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
      .setDimensions(15, 14)
      // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
      .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
      .setStartPose(new Pose2d(48, -17, -Math.PI / 2))
      .build();

//    myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(24, -65, Math.PI / 2))
//        .strafeTo(new Vector2d(0,-31))
//      .build()); // 1.65s FINAL TO SPECIMEN BAR START

//    myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -31, Math.PI / 2))
//        .setTangent(-Math.PI / 4)
//        .splineToSplineHeading(new Pose2d(36, -24, -Math.PI / 2), Math.PI / 2)
//        .splineToConstantHeading(new Vector2d(48, -17), -Math.PI / 2)
//      .build()); // 2.80s FINAL TO FIRST ALLY SAMPLE

//    myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(48, -17, -Math.PI / 2))
//        .strafeTo(new Vector2d(58, -17))
//        .build()); // 0.82s FINAL TO SECOND ALLY SAMPLE

//    myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(58, -17, -Math.PI / 2))
//        .strafeToSplineHeading(new Vector2d(58, -25.5), 0)
//        .build()); // 0.94s FINAL TO THIRD ALLY SAMPLE

//    myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(58, -25.5, 0))
//        .strafeToSplineHeading(new Vector2d(58, -50), -Math.PI / 2)
//        .build()); // 1.49s FINAL TO OBSERVATION ZONE

//    myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(58, -50, -Math.PI / 2))
//        .strafeTo(new Vector2d(40.5, -50))
//        .build());

//    myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(40.5, -50, -Math.PI / 2))
//        .strafeToSplineHeading(new Vector2d(0, -31), Math.PI / 2)
//        .build()); // 2.36s FINAL TO BAR FROM ZONE

//    myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -31, Math.PI / 2))
//        .strafeToSplineHeading(new Vector2d(40.5, -50), -Math.PI / 2)
//        .build()); // 2.36s FINAL TO ZONE FROM BAR

//    myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -31, Math.PI / 2))
//        .strafeTo(new Vector2d(64.5, -65))
//        .build()); // 2.38s FINAL TO PARK FROM BAR

    meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
      .setDarkMode(true)
      .setBackgroundAlpha(0.95f)
      .addEntity(myBot)
      .start();
  }
}