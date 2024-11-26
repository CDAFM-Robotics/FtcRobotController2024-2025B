package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class RRTrajectories {

  Robot myRobot;

  MecanumDrive drive;

  // Observation Side

  public Action rightStartToBar;
  public Action barToAllySample;
  public Action sampleToSampleRight;
  public Action sampleToSampleRightWall;
  public Action rightWallAllySampleToObservationZone;
  public Action observationZoneToSpecimenWallPos;
  public Action specimenWallPosToBar;
  public Action barToSpecimenWallPos;
  public Action barToParkCorner;

  // TODO: Sample Side

  public RRTrajectories(Robot robot) {
    myRobot = robot;
  }
  public void initTrajectories() {

    drive = new MecanumDrive(myRobot.myOpMode.hardwareMap, new Pose2d(0, 0 ,0));

    rightStartToBar = drive.actionBuilder(new Pose2d(24, -65, Math.PI / 2))
      .strafeTo(new Vector2d(0,-31))
      .build();

    barToAllySample = drive.actionBuilder(new Pose2d(0, -31, Math.PI / 2))
      .setTangent(-Math.PI / 4)
      .splineToSplineHeading(new Pose2d(36, -24, -Math.PI / 2), Math.PI / 2)
      .splineToConstantHeading(new Vector2d(48, -17), -Math.PI / 2)
      .build();

    sampleToSampleRight = drive.actionBuilder(new Pose2d(48, -17, -Math.PI / 2))
        .strafeTo(new Vector2d(58, -17))
        .build();

    sampleToSampleRightWall = drive.actionBuilder(new Pose2d(58, -17, -Math.PI / 2))
        .strafeToSplineHeading(new Vector2d(58, -25.5), 0)
        .build();

    rightWallAllySampleToObservationZone = drive.actionBuilder(new Pose2d(58, -25.5, 0))
        .strafeToSplineHeading(new Vector2d(58, -50), -Math.PI / 2)
        .build();

    observationZoneToSpecimenWallPos = drive.actionBuilder(new Pose2d(58, -50, -Math.PI / 2))
        .strafeTo(new Vector2d(40.5, -50))
        .build();

    specimenWallPosToBar = drive.actionBuilder(new Pose2d(40.5, -50, -Math.PI / 2))
        .strafeToSplineHeading(new Vector2d(0, -31), Math.PI / 2)
        .build();

    barToSpecimenWallPos = drive.actionBuilder(new Pose2d(0, -31, Math.PI / 2))
        .strafeToSplineHeading(new Vector2d(40.5, -50), -Math.PI / 2)
        .build();

    barToParkCorner = drive.actionBuilder(new Pose2d(0, -31, Math.PI / 2))
        .strafeTo(new Vector2d(64.5, -65))
        .build();


  }
}
