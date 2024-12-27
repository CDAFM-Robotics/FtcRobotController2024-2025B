package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.Robot;

@TeleOp(name = "Driver Control TeleOp", group = "0competition")
public class DriverControlledOpMode extends LinearOpMode {
  //Create an instant of the Robot Class
  Robot robot = new Robot(this);

  // Variables for Functions
  public Gamepad currentGamepad1 = new Gamepad();
  public Gamepad previousGamepad1 = new Gamepad();
  public Gamepad currentGamepad2 = new Gamepad();
  public Gamepad previousGamepad2 = new Gamepad();

  @Override
  public void runOpMode() {

    telemetry.addData("Status", "Initializing...");
    // Initialize the robot
    robot.initializeDevices();

    telemetry.addData("Status", "Initialized");
    telemetry.addData("Rotation Motor ", "Target: %d, Current: %d", robot.getSlideRotationMotorTargetPosition(), robot.getSlideRotationMotorCurrentPosition());
    telemetry.addData("Extension Motor ", "Target: %s, Current: %d", robot.getSlideExtensionMotorTargetPosition(), robot.getSlideExtensionMotorCurrentPosition());
    telemetry.update();

    waitForStart();

    while (opModeIsActive()) {

      int p = 0;

      telemetry.addData("Status", "Running");
      // Update the gamepads.
      // Using previousGamepad and currentGampad to detect one button push.
      previousGamepad1.copy(currentGamepad1);
      currentGamepad1.copy(gamepad1);
      previousGamepad2.copy(currentGamepad2);
      currentGamepad2.copy(gamepad2);

      // Drive Train Control
      if (currentGamepad1.left_stick_button && !previousGamepad1.left_stick_button) {
        robot.toggleDriveSpeed();
      }
      telemetry.addData("Speed", robot.getDriveSpeed());
      robot.setMotorPowers(Math.pow(currentGamepad1.left_stick_x, 3), Math.pow(currentGamepad1.left_stick_y, 3), Math.pow(currentGamepad1.right_stick_x, 3), 0);

      /*************************
       *  Driver Arm Control   *
       *************************/
      // Driver Arm Rotation Control
      if (currentGamepad2.left_stick_y < 0) {
        robot.setSlideRotationMotorPower(Math.pow(currentGamepad2.left_stick_y,2));
        robot.setSlideRotationMotorTargetPosition(robot.ARM_ROT_DROP_OFF_SAMPLES);

      }
      else if (currentGamepad2.left_stick_y > 0) {
        robot.setSlideRotationMotorPower(Math.pow(currentGamepad2.left_stick_y,2));
        p = (int) (robot.ARM_EXT_DROP_TOP_BASKET / ((robot.SECONDS_DOWN_FAST / robot.CYCLE_TIME
                - robot.SECONDS_DOWN_SLOW / robot.CYCLE_TIME )
                * currentGamepad2.left_stick_y + robot.SECONDS_DOWN_SLOW / robot.CYCLE_TIME));
        robot.setSlideRotationMotorTargetPosition(robot.slideRotationMotor.getTargetPosition() - p);
        telemetry.addData("left stick Rotation Motor ", "Target: %d, Current: %d", robot.getSlideRotationMotorTargetPosition(), robot.getSlideRotationMotorCurrentPosition());
      }
      else {
        if (currentGamepad2.left_stick_y == 0 && previousGamepad2.left_stick_y != 0 ) {
          robot.setSlideRotationMotorPower(robot.ARM_ROT_POWER_FULL);
          robot.setSlideRotationMotorTargetPosition(robot.getSlideRotationMotorCurrentPosition());
        }
      }

      // Driver Arm Extension Control
      if (currentGamepad2.right_stick_y < 0) {
        robot.setSlideExtensionMotorPower(currentGamepad2.right_stick_y * Math.pow(currentGamepad2.right_stick_y,2));
        robot.setSlideExtMotorTargetPosWithLimit(robot.ARM_EXT_DROP_TOP_BASKET);
      }
      else if (currentGamepad2.right_stick_y > 0) {
        robot.setSlideExtensionMotorPower(currentGamepad2.right_stick_y * Math.pow(currentGamepad2.right_stick_y,2));
        robot.setSlideExtensionMotorTargetPosition(robot.ARM_EXT_INIT);
      }
      else {
        if (currentGamepad2.right_stick_y == 0 && previousGamepad2.right_stick_y != 0) {
          robot.setSlideExtensionMotorPower(robot.ARM_ROT_POWER_FULL);
          robot.setSlideExtensionMotorTargetPosition(robot.getSlideExtensionMotorCurrentPosition());
        }
      }

      /*************************
       *  Driver Hand Control  *
       *************************/
      // Toggle the finger position
      if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
        robot.toggleClawGrabPosition();
      }

      // Wrist joint move up
      if (currentGamepad2.b && !previousGamepad2.b) {
        robot.clawPanServoUp();
      }

      // Wrist joint move up
      if (currentGamepad2.a && !previousGamepad2.a) {
        robot.clawPanServoDown();
      }

      if (currentGamepad2.back && !previousGamepad2.back) {
        robot.toggleClawRotation();
      }

      /*************************
       *       Macros          *
       *************************/
      // pick up
      if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
        robot.pickupPosition();
      }

      // drop in top basket
      if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
        robot.dropTopBasket();
      }

      // drop in bottom basket
      if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
        robot.dropBottomBasket();
      }

      // pick up specimen for side wall
      if (currentGamepad2.y && !previousGamepad2.y) {
        robot.wallPickup();
      }

      // get ready to hang specimen on top bar
      if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
        robot.topSpecimenBar();
      }

      // drive position
      if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
        robot.drivePosition();
      }

      // TODO hang the robot, available after 1:30 in teleOp
      // get ready to hang the robot

      // hang the robot

      /*********************************************************
       *  move the arm and hand according to target positions  *
       *********************************************************/
      robot.moveArmToPosition();
      robot.moveHandToPosition();

      telemetry.addData("Rotation Motor ", "Target: %d, Current: %d", robot.getSlideRotationMotorTargetPosition(), robot.getSlideRotationMotorCurrentPosition());
      telemetry.addData("Rotation Motor ", "power: %f, p per cycle: %d", robot.slideRotationMotor.getPower(), p);
      telemetry.addData("Extension Motor ", "Target: %s, Current: %d", robot.getSlideExtensionMotorTargetPosition(), robot.getSlideExtensionMotorCurrentPosition());
      telemetry.addData("Extension Motor ", "power: %f", robot.slideExtensionMotor.getPower());
      telemetry.addData("Fingers servo ", "position %f", robot.clawGrabServo.getPosition());
      telemetry.addData("pan servo ", "position %f", robot.clawPanServo.getPosition());
      telemetry.addData("rotation servo ", "position %f", robot.clawRotateServo.getPosition());

      telemetry.update();
      }
  }
}