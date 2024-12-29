package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;

@TeleOp(name = "Driver Control TeleOp", group = "0competition")
public class DriverControlledOpMode extends LinearOpMode {
  public enum ArmHandState {
    ARM_HOLD,
    DRIVER_CONTROL,
    ARM_PICKUP,
    HAND_PICKUP_SEQ,
    HAND_PICKUP_SEQ_1,
    HAND_PICKUP_SEQ_2,
    HAND_PICKUP_SEQ_3,
    ARM_DROP_TOP,
    HAND_DROP_TOP,
    HAND_DROP_TOP_1,
    ARM_HAND_DRIVE,
    ARM_TOP_SPECIMEN,
    ARM_TOP_SPECIMEN_PULL
  };

  ArmHandState armHandState;
  ArmHandState prevArmHandState;

  ElapsedTime timer = new ElapsedTime();

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

    armHandState = ArmHandState.ARM_HOLD;

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
        prevArmHandState = armHandState;
        armHandState = ArmHandState.DRIVER_CONTROL;
      }
      else if (currentGamepad2.left_stick_y > 0) {
        robot.setSlideRotationMotorPower(Math.pow(currentGamepad2.left_stick_y,2));
        // Try to set the target position to avoid arm drop due to gravity
        p = (int) (robot.ARM_EXT_DROP_TOP_BASKET / ((robot.SECONDS_DOWN_FAST / robot.CYCLE_TIME
                - robot.SECONDS_DOWN_SLOW / robot.CYCLE_TIME )
                * currentGamepad2.left_stick_y + robot.SECONDS_DOWN_SLOW / robot.CYCLE_TIME));
        robot.setSlideRotationMotorTargetPosition(robot.slideRotationMotor.getTargetPosition() - p);
        prevArmHandState = armHandState;
        armHandState = ArmHandState.DRIVER_CONTROL;
        telemetry.addData("left stick Rotation Motor ", "Target: %d, Current: %d", robot.getSlideRotationMotorTargetPosition(), robot.getSlideRotationMotorCurrentPosition());
      }
      else {
        if (currentGamepad2.left_stick_y == 0 && previousGamepad2.left_stick_y != 0 ) {
          robot.setSlideRotationMotorPower(robot.ARM_ROT_POWER_FULL);
          robot.setSlideRotationMotorTargetPosition(robot.getSlideRotationMotorCurrentPosition());
          prevArmHandState = armHandState;
          armHandState = ArmHandState.ARM_HOLD;
        }
      }

      // Driver Arm Extension Control
      if (currentGamepad2.right_stick_y < 0) {
        robot.setSlideExtensionMotorPower(currentGamepad2.right_stick_y * Math.pow(currentGamepad2.right_stick_y,2));
        robot.setSlideExtMotorTargetPosWithLimit(robot.ARM_EXT_DROP_TOP_BASKET);
        prevArmHandState = armHandState;
        armHandState = ArmHandState.DRIVER_CONTROL;
      }
      else if (currentGamepad2.right_stick_y > 0) {
        robot.setSlideExtensionMotorPower(currentGamepad2.right_stick_y * Math.pow(currentGamepad2.right_stick_y,2));
        robot.setSlideExtensionMotorTargetPosition(robot.ARM_EXT_INIT);
        prevArmHandState = armHandState;
        armHandState = ArmHandState.DRIVER_CONTROL;
      }
      else {
        if (currentGamepad2.right_stick_y == 0 && previousGamepad2.right_stick_y != 0) {
          robot.setSlideExtensionMotorPower(robot.ARM_ROT_POWER_FULL);
          robot.setSlideExtensionMotorTargetPosition(robot.getSlideExtensionMotorCurrentPosition());
          prevArmHandState = armHandState;
          armHandState = ArmHandState.ARM_HOLD;
        }
      }

      /*************************
       *  Driver Hand Control  *
       *************************/
      // Toggle the finger position
      if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
        robot.toggleClawGrabPosition();
      }

      // the hand will dip down to pick up the sample and then come back up
      if (currentGamepad2.right_trigger > 0) {
        switch (armHandState) {
          case ARM_PICKUP:
            prevArmHandState = armHandState;
            armHandState = ArmHandState.HAND_PICKUP_SEQ;
            // reset the timer
            timer.reset();
            break;
          case ARM_DROP_TOP:
            prevArmHandState = armHandState;
            armHandState = ArmHandState.HAND_DROP_TOP;
            timer.reset();
            break;
          case ARM_TOP_SPECIMEN:
            robot.pullExtToHangSpecimen();
            prevArmHandState = armHandState;
            armHandState = ArmHandState.ARM_TOP_SPECIMEN_PULL;
            break;
          default:
            break;
        }
      }

      // Wrist joint move up 0
      if (currentGamepad2.b && !previousGamepad2.b) {
        robot.clawPanServoUp();
      }

      // Wrist joint move down
      if (currentGamepad2.a && !previousGamepad2.a) {
        robot.clawPanServoDown();
      }

      if (currentGamepad2.back && !previousGamepad2.back) {
        robot.toggleClawRotation();
      }

      /*************************
       *       Macros          *
       *************************/
      // set the arm at pick up position
      if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
        prevArmHandState = armHandState;
        armHandState = ArmHandState.ARM_PICKUP;
        robot.pickupPosition();
      }

      // drop in top basket
      if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
        prevArmHandState = armHandState;
        armHandState = ArmHandState.ARM_DROP_TOP;
        robot.dropTopBasket();
      }

      // drop in bottom basket
      if (currentGamepad2.start && !previousGamepad2.start) {
        robot.dropBottomBasket();
      }

      // pick up specimen for side wall
      if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
        robot.wallPickup();
      }

      // get ready to hang specimen on top bar
      if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
        robot.topSpecimenBar();
        prevArmHandState = armHandState;
        armHandState = ArmHandState.ARM_TOP_SPECIMEN;
      }

      // drive position
      if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
        robot.drivePosition();
        prevArmHandState = armHandState;
        armHandState = ArmHandState.ARM_HAND_DRIVE;

      }

      // TODO hang the robot, available after 1:30 in teleOp
      // get ready to hang the robot

      // hang the robot

      /*********************************************************
       *  move the arm and hand according to target positions  *
       *********************************************************/
      telemetry.addLine(armHandState.toString());
      telemetry.addData("Fingers servo ", "position %f", robot.clawGrabServo.getPosition());
      telemetry.addData("pan servo ", "position %f", robot.clawPanServo.getPosition());

      switch (armHandState) {
        case HAND_PICKUP_SEQ:
          if (!robot.isFingersOpen()) {
            robot.openFingers();
            prevArmHandState = armHandState;
            armHandState = ArmHandState.HAND_PICKUP_SEQ_1;
            break;
          }
          else {
            robot.handPickUpdip();
            prevArmHandState = armHandState;
            armHandState = ArmHandState.HAND_PICKUP_SEQ_2;
            break;
          }
        case HAND_PICKUP_SEQ_1:
          if (timer.seconds() > 0.4) {
            robot.handPickUpdip();
            prevArmHandState = armHandState;
            armHandState = ArmHandState.HAND_PICKUP_SEQ_2;
            timer.reset();
            break;          }
          else {
            break;
          }
        case HAND_PICKUP_SEQ_2:
          if (timer.seconds() > 0.3) {
            robot.closeFingers();
            prevArmHandState = armHandState;
            armHandState = ArmHandState.HAND_PICKUP_SEQ_3;
            timer.reset();
            break;          }
          else {
            break;
          }
        case HAND_PICKUP_SEQ_3:
          if (timer.seconds() > 0.4) {
            robot.handStraight();
            prevArmHandState = armHandState;
            armHandState = ArmHandState.ARM_PICKUP;
            break;
          }
          else {
            break;
          }
        case HAND_DROP_TOP:
          robot.openFingers();
          prevArmHandState = armHandState;
          armHandState = ArmHandState.HAND_DROP_TOP_1;
          break;
        case HAND_DROP_TOP_1:
          if (timer.seconds() > 0.4) {
            robot.handStraight();
            prevArmHandState = armHandState;
            armHandState = ArmHandState.ARM_HOLD;
            break;
          }
          else {
            break;
          }
        case ARM_DROP_TOP:
          robot.moveArmToPosition();
          robot.moveHandToPosition();
          if(robot.armReachedTarget()) {
            robot.handDropTopDip();
            break;
          }
          else
            break;
        case ARM_TOP_SPECIMEN_PULL:
          robot.moveArmToPosition();
          if(robot.armReachedTarget()) {
            robot.openFingers();
            prevArmHandState = armHandState;
            armHandState = ArmHandState.ARM_HOLD;
            break;
          }
          else
            break;
        default:
          robot.moveHandToPosition();
          robot.moveArmToPosition();
          break;
      }

      telemetry.addData("Rotation Motor ", "Target: %d, Current: %d", robot.getSlideRotationMotorTargetPosition(), robot.getSlideRotationMotorCurrentPosition());
      telemetry.addData("Rotation Motor ", "power: %f, p per cycle: %d", robot.slideRotationMotor.getPower(), p);
      telemetry.addData("Extension Motor ", "Target: %s, Current: %d", robot.getSlideExtensionMotorTargetPosition(), robot.getSlideExtensionMotorCurrentPosition());
      telemetry.addData("Extension Motor ", "power: %f", robot.slideExtensionMotor.getPower());
      telemetry.addData("Fingers servo ", "position %f", robot.clawGrabServo.getPosition());
      telemetry.addData("pan servo ", "position %f", robot.clawPanServo.getPosition());
      telemetry.addData("rotation servo ", "position %f", robot.clawRotateServo.getPosition());
      telemetry.addLine(armHandState.toString());

      telemetry.update();
      }
  }
}