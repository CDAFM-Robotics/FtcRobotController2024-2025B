package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.RRTrajectories;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Observation Zone Autonomous", group = "0Competition")
public class AutoObservationSideOpMode extends LinearOpMode {

  RRTrajectories rrTrajectories;
  Robot robot;

  Action[] trajectories;

  @Override
  public void runOpMode() {
    robot = new Robot(this);
    rrTrajectories = new RRTrajectories(this.hardwareMap);

    robot.initializeArmDevices();
    rrTrajectories.initTrajectories();
    trajectories = rrTrajectories.getRightSideTrajectories();


    waitForStart();

    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_HANG);
    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_HANG);
    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_HANG);

    Actions.runBlocking(trajectories[0]);

    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_HANG - 500);
    while (Math.abs(robot.slideRotationMotor.getCurrentPosition() - robot.slideRotationMotor.getTargetPosition()) > 5) {
      robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_HANG - 500);
    }

    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_OPEN);
    robot.setClawPanServoPosition(0.5);
    robot.slideRotationMotor.setTargetPosition(1200);
    robot.slideExtensionMotor.setTargetPosition(0);
    robot.setClawRotateServoPosition(0.35);

    Actions.runBlocking(trajectories[1]);

    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_OPEN);
    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_PICKUP);
    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_PICKUP);
    robot.setClawPanServoPosition(0.4);
    robot.setClawRotateServoPosition(0.35);

    while (Math.abs(robot.slideExtensionMotor.getCurrentPosition() - robot.slideExtensionMotor.getTargetPosition()) > 10 || Math.abs(robot.slideRotationMotor.getCurrentPosition() - robot.slideRotationMotor.getTargetPosition()) > 10) {
      continue;
    }

    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_PICKUP);

    sleep(500);

    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_CLOSED);

    sleep(250);

    Actions.runBlocking(trajectories[2]);

    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_OPEN);

    Actions.runBlocking(trajectories[3]);
    Actions.runBlocking(trajectories[4]);
    Actions.runBlocking(trajectories[5]);
    Actions.runBlocking(trajectories[6]);
    Actions.runBlocking(trajectories[7]);
    Actions.runBlocking(trajectories[8]);
    Actions.runBlocking(trajectories[9]);
    Actions.runBlocking(trajectories[10]);
    Actions.runBlocking(trajectories[11]);
    Actions.runBlocking(trajectories[12]);
    Actions.runBlocking(trajectories[13]);
  }
}
