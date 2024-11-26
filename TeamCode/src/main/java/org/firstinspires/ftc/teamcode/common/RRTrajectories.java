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

  public Action[] rightSideTrajectories;

  // TODO: Sample Side

  public Action leftStartToNet;
  public Action netToFirstYellowSample;
  public Action firstYellowSampleToNet;
  public Action netToSecondYellowSample;
  public Action secondYellowSampleToNet;
  public Action netToThirdYellowSampleWall;
  public Action thirdYellowSampleWallToNet;
  public Action netToAscentZone;

  public Action[] leftSideTrajectories;

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

    rightSideTrajectories = new Action[] {rightStartToBar, barToAllySample, sampleToSampleRight, sampleToSampleRightWall, rightWallAllySampleToObservationZone, observationZoneToSpecimenWallPos, specimenWallPosToBar, barToSpecimenWallPos, barToParkCorner};

    leftStartToNet = drive.actionBuilder(new Pose2d(-39, -65, Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(-54, -54), -Math.PI * (3.0 / 4.0))
      .build();

    netToFirstYellowSample = drive.actionBuilder(new Pose2d(-54, -54, -Math.PI * (3.0/4.0)))
        .strafeToSplineHeading(new Vector2d(-48, -35), Math.PI / 2)
        .build();

    firstYellowSampleToNet = drive.actionBuilder(new Pose2d(-48, -35, Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(-54, -54), -Math.PI * (3.0 / 4.0))
      .build();

    netToSecondYellowSample = drive.actionBuilder(new Pose2d(-54, -54, -Math.PI * (3.0 / 4.0)))
      .strafeToSplineHeading(new Vector2d(-60, -35), Math.PI / 2)
      .build();

    secondYellowSampleToNet = drive.actionBuilder(new Pose2d(-60, -35, Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(-54, -54), -Math.PI * (3.0 / 4.0))
      .build();

    netToThirdYellowSampleWall = drive.actionBuilder(new Pose2d(-54, -54, -Math.PI * (3.0 / 4.0)))
      .strafeToSplineHeading(new Vector2d(-60, -26), Math.PI)
      .build();

    thirdYellowSampleWallToNet = drive.actionBuilder(new Pose2d(-60, -26, Math.PI))
      .strafeToSplineHeading(new Vector2d(-54, -54), -Math.PI * (3.0 / 4.0))
      .build();

    netToAscentZone = drive.actionBuilder(new Pose2d(-54, -54, -Math.PI * (3.0 / 4.0)))
      .setTangent(Math.PI / 2)
      .splineToSplineHeading(new Pose2d(-54, -12, 0), Math.PI / 2)
      .splineToConstantHeading(new Vector2d(-24, 0), 0)
      .build();

    leftSideTrajectories = new Action[] {leftStartToNet, netToFirstYellowSample, firstYellowSampleToNet, netToSecondYellowSample, secondYellowSampleToNet, netToThirdYellowSampleWall, thirdYellowSampleWallToNet, netToAscentZone};
  }
}
