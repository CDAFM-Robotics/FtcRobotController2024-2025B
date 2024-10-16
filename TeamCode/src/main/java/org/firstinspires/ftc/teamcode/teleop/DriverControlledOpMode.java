package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.Robot;

public class DriverControlledOpMode extends LinearOpMode {
  Robot robot = new Robot(this);

  double clawPosition = robot.CLAW_GRAB_POSITION_OPEN;
  double clawRotatePosition;
  @Override
  public void runOpMode() {

    telemetry.addData("Status", "Initializing");
    telemetry.update();

    robot.initializeDevices();

    telemetry.addData("Status", "Initialized");
    telemetry.update();

    waitForStart();
    while (opModeIsActive()) {

      telemetry.addData("Status", "Running");

      robot.updateGamepads();


      // Drive Train Control
      robot.setMotorPowers(Math.pow(gamepad1.left_stick_x, 3), Math.pow(gamepad1.left_stick_y, 3), Math.pow(gamepad1.right_stick_x, 3), 0);

      // Arm Control
      robot.slideExtensionMotor.setPower(Math.pow(gamepad2.right_stick_y, 3));
      robot.slideRotationMotor.setPower(Math.pow(gamepad2.left_stick_y, 3));

      // Hand Control
      if (robot.currentGamepad2.right_bumper && !robot.previousGamepad2.right_bumper) {
        if (clawPosition == robot.CLAW_GRAB_POSITION_OPEN) {
          clawPosition = robot.CLAW_GRAB_POSITION_CLOSED;
        } else {
          clawPosition = robot.CLAW_GRAB_POSITION_OPEN;
        }
      }
      robot.clawGrabServo.setPosition(clawPosition);

      if (robot.currentGamepad2.a && !robot.previousGamepad2.a) {
        clawRotatePosition += 0.1;
      }
      if (robot.currentGamepad2.b && !robot.previousGamepad2.b) {
        clawRotatePosition -= 0.1;
      }

      // Linear Actuator Control
      robot.linearActuatorLeftMotor.setPower((gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0));
      robot.linearActuatorRightMotor.setPower((gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0));
    }
  }
}