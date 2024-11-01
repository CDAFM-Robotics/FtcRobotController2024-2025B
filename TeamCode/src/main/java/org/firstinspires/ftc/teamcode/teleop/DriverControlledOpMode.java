package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;

@TeleOp(name = "Driver Control TeleOp", group = "0competition")
public class DriverControlledOpMode extends LinearOpMode {
  Robot robot = new Robot(this);

  double lStickY2;
  double rStickY2;

double clawOpenPosition = robot.CLAW_GRAB_POSITION_CLOSED;
  double clawPanPosition;
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
      robot.setMotorPowers(Math.pow(gamepad1.left_stick_x, 3), Math.pow(gamepad1.left_stick_y, 3), Math.pow(gamepad1.right_stick_x, 3), 0);

      // Arm Control

      robot.setArmPosition(robot.convertTicksToDegrees312RPM(robot.slideExtensionMotor.getCurrentPosition()) * Robot.CONVERT_DEGREES_INCHES_SLIDE, robot.slideRotationMotor.getCurrentPosition() / 14.6697222222 - 25.1, gamepad2.right_stick_y, gamepad2.left_stick_y);

      // Hand Control
      if (robot.currentGamepad2.right_bumper && !robot.previousGamepad2.right_bumper) {
        if (clawOpenPosition == robot.CLAW_GRAB_POSITION_OPEN) {
          clawOpenPosition = robot.CLAW_GRAB_POSITION_CLOSED;
        } else {
          clawOpenPosition = robot.CLAW_GRAB_POSITION_OPEN;
        }
      }
      telemetry.addData("Grab Servo", "%.5f", clawOpenPosition);
      robot.setClawGrabServoPosition(clawOpenPosition);

      if (robot.currentGamepad2.left_trigger != 0 && robot.previousGamepad2.left_trigger == 0) {
        clawPanPosition += 0.025;
      }
      if (robot.currentGamepad2.right_trigger != 0 && robot.previousGamepad2.right_trigger == 0) {
        clawPanPosition -= 0.025;
      }
      telemetry.addData("Pan Servo", "%.5f", clawPanPosition);
      robot.setClawPanServoPosition(clawPanPosition);


      // Linear Actuator Control
      // robot.linearActuatorLeftMotor.setPower((gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0));
      // robot.linearActuatorRightMotor.setPower((gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0));
      telemetry.update();
    }
  }
}