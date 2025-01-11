package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;

@Autonomous(name = "Autonomous", group = "zArchived")
@Disabled
public class BasicNoRRAutonomousOpMode extends LinearOpMode {
  Robot robot = new Robot(this);
  ElapsedTime elapsedTime = new ElapsedTime();
  @Override
  public void runOpMode() {
    robot.initializeDevices();
    waitForStart();
    elapsedTime.reset();
    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_DRIVE);
    while (opModeIsActive() && elapsedTime.milliseconds() < 5000) {
      robot.setMotorPowers(0, -1, 0, 0, 0.2);
    }
    robot.setMotorPowers(0, 0, 0, 0, 0.2);
    while (opModeIsActive() && Math.abs(robot.slideRotationMotor.getCurrentPosition() - Robot.ARM_ROT_AUTO_HANG) > 10) {
      robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_HANG);
    }
    while (opModeIsActive() && Math.abs(robot.slideExtensionMotor.getCurrentPosition() - Robot.ARM_EXT_AUTO_HANG) > 10) {
      robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_HANG);
    }
    elapsedTime.reset();
    while (opModeIsActive() && elapsedTime.milliseconds() < 500) {
      robot.clawPanServo.setPosition(Robot.CLAW_PAN_POSITION_AUTO_HANG);
    }
    elapsedTime.reset();
    while (opModeIsActive() && elapsedTime.milliseconds() < 500) {
      robot.slideExtensionMotor.setTargetPosition(0);
      robot.clawPanServo.setPosition(1);
      robot.clawGrabServo.setPosition(Robot.CLAW_GRAB_POSITION_CLOSED);
    }
    elapsedTime.reset();
    while (opModeIsActive() && elapsedTime.milliseconds() < 3000){
      robot.setMotorPowers(0, 1, 0, 0, 0.2);
      robot.slideRotationMotor.setTargetPosition(1000);
      robot.clawGrabServo.setPosition(Robot.CLAW_GRAB_POSITION_OPEN);
      robot.clawPanServo.setPosition(0);
    }
    elapsedTime.reset();
    while (opModeIsActive() && elapsedTime.milliseconds() < 500) {
      robot.setMotorPowers(0, 1, 0, 0, 0.2);
      robot.slideRotationMotor.setTargetPosition(450);
      robot.clawGrabServo.setPosition(Robot.CLAW_GRAB_POSITION_CLOSED);

    }
    elapsedTime.reset();
    while (opModeIsActive() && elapsedTime.milliseconds() < 250) {
      robot.setMotorPowers(0, 0, 0, 0, 0);
    }
    while (opModeIsActive() && elapsedTime.milliseconds() < 2500) {
      robot.setMotorPowers(1, 0, 0, 0, 0.5);
    }
  }
}