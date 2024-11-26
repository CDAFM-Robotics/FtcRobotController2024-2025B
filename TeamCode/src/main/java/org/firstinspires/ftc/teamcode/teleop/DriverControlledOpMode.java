package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;

@TeleOp(name = "Driver Control TeleOp", group = "0competition")
public class DriverControlledOpMode extends LinearOpMode {
  Robot robot = new Robot(this);

  double driveSpeed = Robot.DRIVE_TRAIN_SPEED_FAST;

  double clawOpenPosition = Robot.CLAW_GRAB_POSITION_CLOSED;
  double clawPanPosition;
  double clawRotatePosition = Robot.CLAW_ROTATE_POSITION_STRAIGHT;
  @Override
  public void runOpMode() {

    telemetry.addData("Status", "Initializing...");

    robot.initializeDevices();

    telemetry.addData("Status", "Initialized");
    telemetry.addData("Rotation", "Arm Motor decoder: %d", robot.slideRotationMotor.getCurrentPosition());
    telemetry.addData("Rotation Arm Motor", "run mode: %s", robot.slideRotationMotor.getMode().toString());
    telemetry.update();

    waitForStart();
    while (opModeIsActive()) {

      telemetry.addData("Status", "Running");

      robot.updateGamepads();


      // Drive Train Control
      if (robot.currentGamepad1.left_stick_button && !robot.previousGamepad1.left_stick_button) {
        driveSpeed = driveSpeed == Robot.DRIVE_TRAIN_SPEED_FAST ? Robot.DRIVE_TRAIN_SPEED_SLOW : Robot.DRIVE_TRAIN_SPEED_FAST;
      }
      telemetry.addData("Speed", driveSpeed);
      robot.setMotorPowers(Math.pow(gamepad1.left_stick_x, 3), Math.pow(gamepad1.left_stick_y, 3), Math.pow(gamepad1.right_stick_x, 3), 0, driveSpeed);

      // Arm Control

      robot.setArmPosition(robot.convertTicksToDegrees117RPM(robot.slideExtensionMotor.getCurrentPosition()) * Robot.CONVERT_DEGREES_INCHES_SLIDE, robot.slideRotationMotor.getCurrentPosition() / 14.6697222222 - 25.1, gamepad2.right_stick_y, gamepad2.left_stick_y);

      // Hand Control
      robot.setClawPosition();

      telemetry.update();
    }
  }
}