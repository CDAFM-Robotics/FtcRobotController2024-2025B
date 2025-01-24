package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.RRPushTrajectories;
import org.firstinspires.ftc.teamcode.common.RRTrajectories;
import org.firstinspires.ftc.teamcode.common.Robot;

@Autonomous(name = "Net Zone Autonomous", group = "0Competition")
public class AutoNetSideOpMode extends LinearOpMode {

  RRPushTrajectories rrPushTrajectories;
  Robot robot;

  Action[] trajectories;

  @Override
  public void runOpMode() {
    robot = new Robot(this);
    rrPushTrajectories = new RRPushTrajectories(this.hardwareMap);

    robot.initializeArmDevices();
    rrPushTrajectories.initTrajectories();
    trajectories = rrPushTrajectories.getLeftSideTrajectories();


    waitForStart();

    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_DROP_OFF_SAMPLES);
    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_DROP_TOP_BASKET);
    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_STRAIGHT);
    sleep(10000);
    Actions.runBlocking(trajectories[0]);

    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_DROP_DIP);

    sleep(300);

    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_OPEN);

    sleep(100);

    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_PICKUP);
    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_PICKUP);
    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_STRAIGHT);

    Actions.runBlocking(trajectories[1]);
    /*
    Actions.runBlocking(trajectories[2]);
    Actions.runBlocking(trajectories[3]);
    Actions.runBlocking(trajectories[4]);
    Actions.runBlocking(trajectories[5]);
    Actions.runBlocking(trajectories[6]);
    Actions.runBlocking(trajectories[7]);

     */
  }
}
