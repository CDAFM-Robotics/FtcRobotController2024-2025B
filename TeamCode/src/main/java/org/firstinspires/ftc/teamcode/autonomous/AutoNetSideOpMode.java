package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.RRTrajectories;
import org.firstinspires.ftc.teamcode.common.Robot;

@Autonomous(name = "Net Zone Autonomous", group = "0Competition")
public class AutoNetSideOpMode extends LinearOpMode {

  RRTrajectories rrTrajectories;
  Robot robot;

  Action[] trajectories;

  @Override
  public void runOpMode() {
    robot = new Robot(this);
    rrTrajectories = new RRTrajectories(this.hardwareMap);

    robot.initializeArmDevices();
    rrTrajectories.initTrajectories();
    trajectories = rrTrajectories.getLeftSideTrajectories();


    waitForStart();

    robot.slideRotationMotor.setTargetPosition(200);

    Actions.runBlocking(trajectories[0]);
    Actions.runBlocking(trajectories[1]);
    Actions.runBlocking(trajectories[2]);
    Actions.runBlocking(trajectories[3]);
    Actions.runBlocking(trajectories[4]);
    Actions.runBlocking(trajectories[5]);
    Actions.runBlocking(trajectories[6]);
    Actions.runBlocking(trajectories[7]);
  }
}
