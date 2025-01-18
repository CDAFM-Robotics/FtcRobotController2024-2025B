package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.RRPushTrajectories;
import org.firstinspires.ftc.teamcode.common.RRTrajectories;
import org.firstinspires.ftc.teamcode.common.Robot;

@Autonomous(name = "Observation Zone Push Autonomous", group = "Testing")
public class AutoObservationPushSideOpMode extends LinearOpMode {

  RRPushTrajectories rrTrajectories;
  Robot robot;

  Action[] trajectories;

  @Override
  public void runOpMode() {
    robot = new Robot(this);
    rrTrajectories = new RRPushTrajectories(this.hardwareMap);

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
    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_DRIVE);
    robot.slideRotationMotor.setTargetPosition(1200);
    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_HANG - 300);
    robot.setClawRotateServoPosition(Robot.CLAW_ROTATE_POSITION_STRAIGHT);

    sleep(500);

    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_OPEN);
    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_DRIVE);
    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_HANG_TOP_SPECIMEN);
    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_DRIVE);
    robot.setClawRotateServoPosition(Robot.CLAW_ROTATE_POSITION_STRAIGHT);

    Actions.runBlocking(trajectories[1]);



/*


    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_OPEN);
    robot.slideRotationMotor.setPower(0.5);
    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_PICKUP);
    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_PICKUP);
    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_PICKUP);
    robot.setClawRotateServoPosition(Robot.CLAW_ROTATE_POSITION_AUTO_PICKUP);

    while (Math.abs(robot.slideExtensionMotor.getCurrentPosition() - robot.slideExtensionMotor.getTargetPosition()) > 10 || Math.abs(robot.slideRotationMotor.getCurrentPosition() - robot.slideRotationMotor.getTargetPosition()) > 10) {
      continue;
    }
    pickUpSampleAuto();

    sleep(250);

    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_PICKUP + 350);
    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_DROP_OBSERVE);

    sleep(100);

    Actions.runBlocking(trajectories[2]);

    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_OPEN);

    sleep(250);

    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_PICKUP_WALL);
    robot.slideExtensionMotor.setTargetPosition(0);
    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_OPEN);
    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_PICKUP_WALL);
    robot.setClawRotateServoPosition(Robot.CLAW_ROTATE_POSITION_STRAIGHT);

    sleep(5000);

    Actions.runBlocking(trajectories[7]);

    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_CLOSED);

    sleep(500);

    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_PICKUP_WALL + 200);

    sleep(500);

    robot.slideRotationMotor.setPower(1);

    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_HANG);
    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_HANG);
    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_HANG);

    Actions.runBlocking(trajectories[8]);

    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_HANG - 500);
    while (Math.abs(robot.slideRotationMotor.getCurrentPosition() - robot.slideRotationMotor.getTargetPosition()) > 5) {
      robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_HANG - 500);
    }

    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_OPEN);
    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_DRIVE);
    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_HANG - 300);
    robot.setClawRotateServoPosition(Robot.CLAW_ROTATE_POSITION_STRAIGHT);

    sleep(1000);

    robot.slideRotationMotor.setTargetPosition(1200);

    Actions.runBlocking(trajectories[13]);

    robot.slideRotationMotor.setPower(0.5);
    robot.slideRotationMotor.setTargetPosition(0);
    robot.slideExtensionMotor.setTargetPosition(0);

    sleep(1000);

    /*

    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_PICKUP + 200);

    Actions.runBlocking(trajectories[3]);

    pickUpSampleAuto();

    Actions.runBlocking(trajectories[4]);

    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_OPEN);

    Actions.runBlocking(trajectories[5]);

    pickUpSampleAuto();

    Actions.runBlocking(trajectories[6]);

    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_OPEN);

    Actions.runBlocking(trajectories[7]);
    Actions.runBlocking(trajectories[8]);
    Actions.runBlocking(trajectories[9]);
    Actions.runBlocking(trajectories[10]);
    Actions.runBlocking(trajectories[11]);
    Actions.runBlocking(trajectories[12]);
    Actions.runBlocking(trajectories[13]);

    robot.slideExtensionMotor.setTargetPosition(0);
    robot.slideRotationMotor.setTargetPosition(0);


     */


  }

  public void pickUpSampleAuto() {
    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_PICKUP);
    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_PICKUP);

    sleep(250);

    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_CLOSED);

    sleep(500);

    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_DRIVE);

    sleep(250);

  }
}
