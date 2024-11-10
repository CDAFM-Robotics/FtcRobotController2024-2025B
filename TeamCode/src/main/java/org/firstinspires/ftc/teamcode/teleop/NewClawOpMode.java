package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;

@TeleOp(name = "Driver Control TeleOp", group = "0competition")
public class NewClawOpMode extends LinearOpMode {
  Robot robot = new Robot(this);

  double lStickY2;
  double rStickY2;

  double clawOpenPosition = robot.CLAW_GRAB_POSITION_OPEN;
  double clawPanPosition;
  double clawRotatePosition;
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
      //robot.setMotorPowers(Math.pow(gamepad1.left_stick_x, 3), Math.pow(gamepad1.left_stick_y, 3), Math.pow(gamepad1.right_stick_x, 3), 0);

      /* ***************************
               Arm Control
      ************************** */

      // Linear slide extension control
      rStickY2 = -gamepad2.right_stick_y * Math.abs(gamepad2.right_stick_y);
      telemetry.addData("Gamepad 2 right StickY", "%.5f", rStickY2);
      robot.slideExtensionMotor.setPower(rStickY2);
      /*if (lStickY2 < 0) {
        //robot.slideExtensionMotor.setTargetPosition(-100);
        robot.slideExtensionMotor.setPower(Math.abs(rStickY2));
      } else if (lStickY2 > 0) {
        //robot.slideExtensionMotor.setTargetPosition(-500);
        robot.slideExtensionMotor.setPower(Math.abs(rStickY2));
      } else {
        //robot.slideExtensionMotor.setTargetPosition(robot.slideExtensionMotor.getCurrentPosition());
        robot.slideExtensionMotor.setPower(Math.abs(rStickY2));
      }
      telemetry.addData("Extension", "Arm Motor decoder: %d", robot.slideRotationMotor.getCurrentPosition());
      telemetry.addData("Extension Arm Motor", "run mode: %s", robot.slideRotationMotor.getMode().toString());
*/
      // Arm rotation
      lStickY2 = -gamepad2.left_stick_y * Math.abs(gamepad2.left_stick_y);
      telemetry.addData("Gamepad 2 Left StickY", "%.5f", lStickY2);
      if (lStickY2 < 0) {
        robot.slideRotationMotor.setTargetPosition(robot.ARM_ROT_PICKUP_SAMPLES);
        robot.slideRotationMotor.setPower(Math.abs(lStickY2));
      } else if (lStickY2 > 0) {
        robot.slideRotationMotor.setTargetPosition(robot.ARM_ROT_DROP_OFF_SAMPLES);
        robot.slideRotationMotor.setPower(Math.abs(lStickY2));
      } else {
        //robot.slideRotationMotor.setTargetPosition(robot.slideRotationMotor.getCurrentPosition());
        robot.slideRotationMotor.setPower(1);
      }
      telemetry.addData("Rotation", "Arm Motor decoder: %d", robot.slideRotationMotor.getCurrentPosition());
      telemetry.addData("Rotation Arm Motor", "run mode: %s", robot.slideRotationMotor.getMode().toString());


      // Hand Control

      //claw grab
      if (robot.currentGamepad2.right_bumper && !robot.previousGamepad2.right_bumper) {
        if (clawOpenPosition == robot.CLAW_GRAB_POSITION_OPEN) {
          clawOpenPosition = robot.CLAW_GRAB_POSITION_CLOSED;
        } else {
          clawOpenPosition = robot.CLAW_GRAB_POSITION_OPEN;
        }
      }
      telemetry.addData("Grab Servot", "%.5f", clawOpenPosition);
      robot.setClawGrabServoPosition(clawOpenPosition);

      //claw pan
      if (robot.currentGamepad2.a && !robot.previousGamepad2.a && clawPanPosition < 0.25) {
        clawPanPosition += 0.05;
      }
      if (robot.currentGamepad2.b && !robot.previousGamepad2.b && clawPanPosition > 0) {
        clawPanPosition -= 0.05;
      }
      telemetry.addData("Pan Servot", "%.5f", clawPanPosition);
      robot.setClawPanServoPosition(clawPanPosition);

      //claw rotate
      if(robot.currentGamepad2.x && !robot.previousGamepad2.x && clawRotatePosition < 0.7){
        clawRotatePosition += 0.05;
      }
      if(robot.currentGamepad2.y && !robot.previousGamepad2.y && clawRotatePosition > 0){
        clawRotatePosition -= 0.05;
      }
      telemetry.addData("Rotate Servo", clawRotatePosition);
      robot.setClawRotateServoPosition(clawRotatePosition);


      // Linear Actuator Control
       robot.linearActuatorLeftMotor.setPower((gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0));
       robot.linearActuatorRightMotor.setPower((gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0));
      telemetry.update();
    }
  }
}