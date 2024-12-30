package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {

  // Motors and devices

  public DcMotor frontLeftMotor = null;
  public DcMotor frontRightMotor = null;
  public DcMotor backLeftMotor = null;
  public DcMotor backRightMotor = null;

  public DcMotor slideExtensionMotor = null;
  public DcMotor slideRotationMotor = null;

  public Servo clawGrabServo = null;
  public Servo clawPanServo = null;
  public Servo clawRotateServo = null;

  LinearOpMode myOpMode;

  // Constants

  public static double CLAW_GRAB_POSITION_CLOSED = 0.0;
  public static double CLAW_GRAB_POSITION_OPEN = 0.5;

  public static double CLAW_ROTATE_POSITION_STRAIGHT = 0.5;
  public static double CLAW_ROTATE_POSITION_AUTO_PICKUP = 0.65;
  public static double CLAW_ROTATE_POSITION_RIGHT = 0.8375;

  public static double CLAW_PAN_TELEOP_INIT = 0.525;
  public static double CLAW_PAN_POSITION_DROP = 0.1994;
  public static double CLAW_PAN_POSITION_DROP_DIP = 0.05744; // don't retract slide with this position!!!
  public static double CLAW_PAN_POSITION_PICKUP = 0.21;
  public static double CLAW_PAN_POSITION_PICKUP_DIP = 0.1244;
  public static double CLAW_PAN_POSITION_PICKUP_WALL = 0.0; // TODO
  public static double CLAW_PAN_POSITION_AUTO_PICKUP_WALL = 0.5494;
  public static double CLAW_PAN_POSITION_TOP_SPECIMEN = 0.0; // TODO
  public static double CLAW_PAN_POSITION_DRIVE = 0.1994;
  public static double CLAW_PAN_POSITION_AUTO_HANG = 0.525;
  public static double CLAW_PAN_POSITION_AUTO_PICKUP = 0.15;
  public static double CLAW_PAN_SPEED = 0.025;

  public static int ARM_EXT_INIT = 0;
  public static int ARM_EXT_DROP_TOP_BASKET = 8085;
  public static int ARM_EXT_DROP_BOTTOM_BASKET = 2222;
  public static int ARM_EXT_HANG_TOP_SPECIMEN = 1300;
  public static int ARM_EXT_PICKUP_SAMPLES = 2293;
  public static int ARM_EXT_DRIVE = 0;
  public static int ARM_EXT_AUTO_HANG = 1366;
  public static int ARM_EXT_AUTO_PICKUP = 2000;
  public static int ARM_EXT_AUTO_DROP_OBSERVE = 4600;

  public static int ARM_ROT_INIT = 0;
  public static int ARM_ROT_DROP_OFF_SAMPLES = 1495;
  public static int ARM_ROT_HANG_TOP_SPECIMEN = 1032;
  public static int ARM_ROT_PICKUP_SAMPLES = 355;
  public static int ARM_ROT_PICKUP_WALL = 282;
  public static int ARM_ROT_AUTO_PICKUP_WALL = 248;
  public static int ARM_ROT_AUTO_HANG = 1251;
  public static int ARM_ROT_AUTO_DRIVE = 1123;
  public static int ARM_ROT_DRIVE = 497;
  public static int ARM_ROT_AUTO_PICKUP = 264;

  public static double ARM_ROT_POWER = 0.5;
  public static double ARM_ROT_POWER_FULL = 1.0;
  public static double ARM_EXT_POWER = 1.0;
  public static double DRIVE_TRAIN_SPEED_FAST = 0.75;
  public static double DRIVE_TRAIN_SPEED_SLOW = 1.0 / 3.0;

  public static double LENGTH_CLAW = 9;
  public static double LENGTH_INSPECTION_FRONT = 30;
  public static double LENGTH_INSPECTION_BACK = 0;
  public static double LENGTH_ARM_EXTENDED = 50;
  public static double LENGTH_ARM_NORMAL = 13.375;

  public static double CONVERT_DEGREES_TICKS_117RPM = 3.95861111111;
  public static double CONVERT_TICKS_DEGREES_117RPM = 1.0 / CONVERT_DEGREES_TICKS_117RPM;
  public static double CONVERT_DEGREES_INCHES_SLIDE = 0.013177365175032795;
  public static double CONVERT_INCHES_DEGREES_SLIDE = 1.0 / CONVERT_DEGREES_INCHES_SLIDE;

  // Arm down control
  public static int SECONDS_DOWN_FAST = 2000;
  public static int SECONDS_DOWN_SLOW = 8000;
  public static int CYCLE_TIME = 5;

  double rotX;
  double rotY;
  double denominator;
  double frontLeftPower;
  double frontRightPower;
  double backLeftPower;
  double backRightPower;

  boolean prevExtending = false;
  boolean prevRotating = false;

  // max extension
  double maxExtension;

  // arm target position
  int slideRotationTargetPosition = 0;
  int slideExtensionTargetPosition = 0;

  // Hand position
  double clawGrabPosition = CLAW_GRAB_POSITION_CLOSED;
  double clawPanPosition = CLAW_PAN_TELEOP_INIT;
  double clawRotatePosition = CLAW_ROTATE_POSITION_STRAIGHT;
  double driveSpeed = DRIVE_TRAIN_SPEED_FAST;


  public Robot(LinearOpMode opMode) {
    myOpMode = opMode;
  }

  public void initializeDevices() {
    frontLeftMotor = myOpMode.hardwareMap.get(DcMotor.class, "frontLeftMotor");
    frontRightMotor = myOpMode.hardwareMap.get(DcMotor.class, "frontRightMotor");
    backLeftMotor = myOpMode.hardwareMap.get(DcMotor.class, "backLeftMotor");
    backRightMotor = myOpMode.hardwareMap.get(DcMotor.class, "backRightMotor");

    frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    initializeArmDevices();
  }

  public void initializeArmDevices() {
    slideExtensionMotor = myOpMode.hardwareMap.get(DcMotor.class, "slideExtensionMotor");
    slideRotationMotor = myOpMode.hardwareMap.get(DcMotor.class, "slideRotationMotor");

    clawGrabServo = myOpMode.hardwareMap.get(Servo.class, "clawGrabServo");
    clawPanServo = myOpMode.hardwareMap.get(Servo.class, "clawPanServo");
    clawRotateServo = myOpMode.hardwareMap.get(Servo.class, "clawRotateServo");

    slideRotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    slideRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slideRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slideRotationMotor.setTargetPosition(ARM_ROT_INIT);
    slideRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    slideRotationMotor.setPower(ARM_ROT_POWER_FULL);


    slideExtensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    slideExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slideExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slideExtensionMotor.setTargetPosition(ARM_EXT_INIT);
    slideExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    slideExtensionMotor.setPower(1);

    clawPanServo.setPosition(CLAW_PAN_TELEOP_INIT);
    clawRotateServo.setPosition(CLAW_ROTATE_POSITION_STRAIGHT);
    clawGrabServo.setPosition(CLAW_GRAB_POSITION_CLOSED);
  }

  public void setMotorPowers(double x, double y, double rx, double heading, double speed) {
    rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
    rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

    // put strafing factors here
    rotX = rotX * 1;
    rotY = rotY * 1;

    denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

    frontLeftPower = (rotY - rotX - rx) / denominator;
    backLeftPower = (rotY + rotX - rx) / denominator;
    frontRightPower = (rotY + rotX + rx) / denominator;
    backRightPower = (rotY - rotX + rx) / denominator;

    frontLeftMotor.setPower(frontLeftPower * speed);
    backLeftMotor.setPower(backLeftPower * speed);
    frontRightMotor.setPower(frontRightPower * speed);
    backRightMotor.setPower(backRightPower * speed);
  }

  public void setMotorPowers(double x, double y, double rx, double heading) {
    rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
    rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

    // put strafing factors here
    rotX = rotX * 1;
    rotY = rotY * 1;

    denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

    frontLeftPower = (rotY - rotX - rx) / denominator;
    backLeftPower = (rotY + rotX - rx) / denominator;
    frontRightPower = (rotY + rotX + rx) / denominator;
    backRightPower = (rotY - rotX + rx) / denominator;

    frontLeftMotor.setPower(frontLeftPower * driveSpeed);
    backLeftMotor.setPower(backLeftPower * driveSpeed);
    frontRightMotor.setPower(frontRightPower * driveSpeed);
    backRightMotor.setPower(backRightPower * driveSpeed);
  }

  // Toggle drive speed
  public void toggleDriveSpeed() {
    driveSpeed = driveSpeed == DRIVE_TRAIN_SPEED_FAST ? DRIVE_TRAIN_SPEED_SLOW : DRIVE_TRAIN_SPEED_FAST;
  }

  // Get drive speed
  public double getDriveSpeed() {
    return driveSpeed;
  }

  // Set Slide Rotation Motor power
  public void setSlideRotationMotorPower(double power) {
    slideRotationMotor.setPower(power);
  }

  // Set Slide Rotation Motor position
  public void setSlideRotationMotorTargetPosition(int position) {
    slideRotationTargetPosition = position;
  }

  // Get Slide Rotation Motor Target position
  public int getSlideRotationMotorTargetPosition() {
    return slideRotationMotor.getTargetPosition();
  }

  // Get Slide Rotation Motor Current position
  public int getSlideRotationMotorCurrentPosition() {
    return slideRotationMotor.getCurrentPosition();
  }

  // Set Slide Extension Motor power
  public void setSlideExtensionMotorPower(double power) {
    slideExtensionMotor.setPower(power);
  }

  // Set Slide Extension Motor position
  // set the extension without soft limit, example 0
  public void setSlideExtensionMotorTargetPosition(int position) {
    if (position > ARM_EXT_DROP_TOP_BASKET)
      slideExtensionTargetPosition = ARM_EXT_DROP_TOP_BASKET;
    else if (position < 0)
      slideExtensionTargetPosition = 0;
    else
      slideExtensionTargetPosition = position;
  }

  public void setSlideExtMotorTargetPosWithLimit(int position) {
    checkSoftLimits(convertTicksToDegrees117RPM(slideExtensionMotor.getCurrentPosition()) * Robot.CONVERT_DEGREES_INCHES_SLIDE,
            slideRotationMotor.getCurrentPosition() / 14.6697222222 - 17.6);

    slideExtensionTargetPosition = convertDegreesToTicks117RPM((maxExtension - LENGTH_ARM_NORMAL) * CONVERT_INCHES_DEGREES_SLIDE);
  }

  // Get Slide Extension Motor Target position
  public int getSlideExtensionMotorTargetPosition() {
    return slideExtensionMotor.getTargetPosition();
  }

  // Get Slide Extension Motor Current position
  public int getSlideExtensionMotorCurrentPosition() {
    return slideExtensionMotor.getCurrentPosition();
  }

  // Set claw Grab Servo position
  public void setClawGrabServoPosition(double grabPosition) {
    clawGrabServo.setPosition(grabPosition);
  }

  // Toggle Finger position
  public void toggleClawGrabPosition() {
    clawGrabPosition = clawGrabServo.getPosition() == CLAW_GRAB_POSITION_CLOSED ? CLAW_GRAB_POSITION_OPEN : CLAW_GRAB_POSITION_CLOSED;
  }

  // Set claw Pan Servo position
  public void setClawPanServoPosition(double panPosition) {
    clawPanPosition = panPosition;
    clawPanServo.setPosition(panPosition);
  }

  // Move claw wrist up
  public void clawPanServoUp() {
    clawPanPosition = (clawPanServo.getPosition() + CLAW_PAN_SPEED) > 1 ? 1 : (clawPanServo.getPosition() + CLAW_PAN_SPEED);
  }

  // Move claw wrist down
  public void clawPanServoDown() {
    clawPanPosition = (clawPanServo.getPosition() - CLAW_PAN_SPEED) < 0 ? 0 : (clawPanServo.getPosition() - CLAW_PAN_SPEED);
  }

  //set claw rotate servo position
  public void setClawRotateServoPosition(double RotatePosition) {
    clawRotateServo.setPosition(RotatePosition);
  }

  public void toggleClawRotation() {
    clawRotatePosition = clawRotateServo.getPosition() == CLAW_ROTATE_POSITION_STRAIGHT ? CLAW_ROTATE_POSITION_RIGHT : CLAW_ROTATE_POSITION_STRAIGHT;
  }

  public void pickupPosition() {
    slideRotationMotor.setPower(ARM_ROT_POWER);
    slideRotationTargetPosition = ARM_ROT_PICKUP_SAMPLES;
    slideExtensionMotor.setPower(ARM_EXT_POWER);
    slideExtensionTargetPosition = ARM_EXT_PICKUP_SAMPLES;
    clawPanPosition = CLAW_PAN_POSITION_PICKUP;
    clawGrabPosition = CLAW_GRAB_POSITION_OPEN;
  }

  public void dropTopBasket() {
    slideRotationMotor.setPower(ARM_ROT_POWER);
    slideRotationTargetPosition = ARM_ROT_DROP_OFF_SAMPLES;
    slideExtensionMotor.setPower(ARM_EXT_POWER);
    slideExtensionTargetPosition = ARM_EXT_DROP_TOP_BASKET;
    clawPanPosition = CLAW_PAN_POSITION_DROP;
  }

  // Bottom Basket Drop
  public void dropBottomBasket() {
    slideRotationMotor.setPower(ARM_ROT_POWER);
    slideRotationTargetPosition = ARM_ROT_DROP_OFF_SAMPLES;
    slideExtensionMotor.setPower(ARM_EXT_POWER);
    slideExtensionTargetPosition = ARM_EXT_DROP_BOTTOM_BASKET;
    clawPanPosition = CLAW_PAN_POSITION_DROP;
  }

  //Wall Pickup
  public void wallPickup () {
    slideRotationMotor.setPower(ARM_ROT_POWER);
    slideRotationTargetPosition = ARM_ROT_PICKUP_WALL;
    slideExtensionMotor.setPower(ARM_EXT_POWER);
    slideExtensionTargetPosition = ARM_EXT_PICKUP_SAMPLES;
    clawPanPosition = CLAW_PAN_POSITION_PICKUP_WALL;
  }

  // Top Specimen Bar
  public void topSpecimenBar () {
    slideRotationMotor.setPower(ARM_ROT_POWER);
    slideRotationTargetPosition = ARM_ROT_HANG_TOP_SPECIMEN;
    clawPanPosition = CLAW_PAN_POSITION_TOP_SPECIMEN;
    slideExtensionMotor.setPower(ARM_EXT_POWER);
    slideExtensionTargetPosition = ARM_EXT_HANG_TOP_SPECIMEN;
  }

  // Bottom Specimen Bar TODO
  public void bottomSpecimenBar () {
  }

  // Drive Position
  public void drivePosition () {
    slideRotationMotor.setPower(ARM_ROT_POWER);
    slideRotationTargetPosition = ARM_ROT_DRIVE;
    slideExtensionMotor.setPower(ARM_EXT_POWER);
    slideExtensionTargetPosition = ARM_EXT_DRIVE;
    clawPanPosition = CLAW_PAN_POSITION_DRIVE;
  }

  // ready to hang on first bar TODO
  public void readyHangFirstBar () {

  }
  // hang on first bar TODO
  public void hangFirstBar () {

  }

  public void moveArmToPosition() {
    // check limit
    checkSoftLimits(convertTicksToDegrees117RPM(slideExtensionMotor.getCurrentPosition()) * Robot.CONVERT_DEGREES_INCHES_SLIDE,
            slideRotationMotor.getCurrentPosition() / 14.6697222222 - 25.1);
    // check to see if the
    int max = convertDegreesToTicks117RPM((maxExtension - LENGTH_ARM_NORMAL) * CONVERT_INCHES_DEGREES_SLIDE);
    if (slideExtensionMotor.getCurrentPosition() > max) {
      slideExtensionTargetPosition = max;
    }

    if (slideExtensionTargetPosition > ARM_EXT_DROP_TOP_BASKET)
      slideExtensionTargetPosition = ARM_EXT_DROP_TOP_BASKET;
    myOpMode.telemetry.addData("3 Rotation Motor ", "Target: %d, Current: %d", getSlideRotationMotorTargetPosition(), getSlideRotationMotorCurrentPosition());

    if (slideExtensionTargetPosition > slideExtensionMotor.getCurrentPosition()) {
      slideRotationMotor.setTargetPosition(slideRotationTargetPosition);
      if (Math.abs(slideRotationTargetPosition - slideRotationMotor.getCurrentPosition()) < 10) {
        slideExtensionMotor.setTargetPosition(slideExtensionTargetPosition);
      }
    } else if (slideExtensionTargetPosition < slideExtensionMotor.getCurrentPosition()) {
      slideExtensionMotor.setTargetPosition(slideExtensionTargetPosition);
      if (Math.abs(slideExtensionTargetPosition - slideExtensionMotor.getCurrentPosition()) < 30) {
        slideRotationMotor.setTargetPosition(slideRotationTargetPosition);
      }
    } else {
      slideRotationMotor.setTargetPosition(slideRotationTargetPosition);
      slideExtensionMotor.setTargetPosition(slideExtensionTargetPosition);
    }
  }

  public void moveHandToPosition() {
    clawGrabServo.setPosition(clawGrabPosition);
    clawPanServo.setPosition(clawPanPosition);
    clawRotateServo.setPosition(clawRotatePosition);
  }

  // check the soft limit for the arm to stay within the 42" length requirement
  public void checkSoftLimits(double armExtension, double armRotation) {
    if (armRotation <= 90) {
      maxExtension = Math.min((LENGTH_INSPECTION_FRONT - LENGTH_CLAW) / Math.cos(Math.toRadians(armRotation)), LENGTH_ARM_EXTENDED);
    } else {
      maxExtension = Math.min((LENGTH_INSPECTION_BACK - LENGTH_CLAW) / Math.cos(Math.toRadians(180 - armRotation)), LENGTH_ARM_EXTENDED);
    }
    myOpMode.telemetry.addData("Arm Rotation degree", armRotation);
    myOpMode.telemetry.addData("Max Extension Soft Limit", maxExtension);
  }

  public int convertDegreesToTicks117RPM(double degrees) {
    return (int) Math.round(degrees * CONVERT_DEGREES_TICKS_117RPM);
  }

  public double convertTicksToDegrees117RPM(int ticks) {
    return ticks * CONVERT_TICKS_DEGREES_117RPM;
  }

/*  public void setMacros() {
    // Macros
    // Bottom Basket Drop
    if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {

      slideRotationMotor.setPower(ARM_ROT_POWER);
      slideRotationTargetPosition = ARM_ROT_DROP_OFF_SAMPLES;
      clawPanPosition = CLAW_PAN_POSITION_DROP;
      slideExtensionMotor.setPower(ARM_EXT_POWER);
      slideExtensionTargetPosition = ARM_EXT_DROP_BOTTOM_BASKET;
    }
    //Top Basket Drop
    if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {

      slideRotationMotor.setPower(ARM_ROT_POWER);
      slideRotationTargetPosition = ARM_ROT_DROP_OFF_SAMPLES;
      clawPanPosition = CLAW_PAN_POSITION_DROP;
      slideExtensionMotor.setPower(ARM_EXT_POWER);
      slideExtensionTargetPosition = ARM_EXT_DROP_TOP_BASKET;
    }
    //Pickup
    if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {

      slideRotationMotor.setPower(ARM_ROT_POWER);
      slideRotationTargetPosition = ARM_ROT_PICKUP_SAMPLES;
      clawPanPosition = CLAW_PAN_POSITION_PICKUP;
      slideExtensionMotor.setPower(ARM_EXT_POWER);
      slideExtensionTargetPosition = ARM_EXT_PICKUP_SAMPLES;
    }
    //Wall Pickup
    if (currentGamepad2.y && !previousGamepad2.y) {

      slideRotationMotor.setPower(ARM_ROT_POWER);
      slideRotationTargetPosition = ARM_ROT_PICKUP_WALL;
      clawPanPosition = CLAW_PAN_POSITION_PICKUP_WALL;
      slideExtensionMotor.setPower(ARM_EXT_POWER);
      slideExtensionTargetPosition = ARM_EXT_PICKUP_SAMPLES;
    }
    // Top Specimen Bar
    if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {

      slideRotationMotor.setPower(ARM_ROT_POWER);
      slideRotationTargetPosition = ARM_ROT_HANG_TOP_SPECIMEN;
      clawPanPosition = CLAW_PAN_POSITION_TOP_SPECIMEN;
      slideExtensionMotor.setPower(ARM_EXT_POWER);
      slideExtensionTargetPosition = ARM_EXT_HANG_TOP_SPECIMEN;
    }
    // Drive Position
    if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {

      slideRotationMotor.setPower(ARM_ROT_POWER);
      slideRotationTargetPosition = ARM_ROT_DRIVE;
      clawPanPosition = CLAW_PAN_POSITION_DRIVE;
      slideExtensionMotor.setPower(ARM_EXT_POWER);
      slideExtensionTargetPosition = ARM_EXT_DRIVE;
    }
  }*/

}
