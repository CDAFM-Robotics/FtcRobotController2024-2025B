package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Set;

public class Robot {

  // Motors and devices

  public DcMotor frontLeftMotor = null;
  public DcMotor frontRightMotor = null;
  public DcMotor backLeftMotor = null;
  public DcMotor backRightMotor = null;

  public DcMotor slideExtensionMotorRight = null;
  public DcMotor slideExtensionMotorLeft = null;
  public DcMotor armRotationMotor = null;

  public Servo clawGrabServo = null;
  public Servo clawPanServo = null;
  public Servo clawRotateServo = null;

  LinearOpMode myOpMode;

  // Constants

  public static double CLAW_GRAB_POSITION_CLOSED = 0.0;
  public static double CLAW_GRAB_POSITION_OPEN = 0.625;

  public static double CLAW_ROTATE_POSITION_STRAIGHT = 0.5;
  public static double CLAW_ROTATE_POSITION_AUTO_PICKUP = 0.65;
  public static double CLAW_ROTATE_POSITION_RIGHT = 0.8375;

  public static double CLAW_PAN_TELEOP_INIT = 0.5;
  public static double CLAW_PAN_POSITION_DROP_DIP = 0.6; // don't retract slide with this position!!!
  public static double CLAW_PAN_POSITION_STRAIGHT = 0.225;
  public static double CLAW_PAN_POSITION_PICKUP_DIP = 0.135;          ;
  public static double CLAW_PAN_POSITION_PICKUP_WALL = 0.5450;
  public static double CLAW_PAN_POSITION_AUTO_PICKUP_WALL = 0.5494;
  public static double CLAW_PAN_POSITION_TOP_SPECIMEN = 0.245;
  public static double CLAW_PAN_POSITION_DRIVE = 0.1994;
  public static double CLAW_PAN_POSITION_HANG_ROBOT = 0.075;
  public static double CLAW_PAN_POSITION_AUTO_HANG = 0.2;
  public static double CLAW_PAN_POSITION_AUTO_DROP_DIP = 0.6;
  public static double CLAW_PAN_POSITION_AUTO_STRAIGHT = 0.225;
  public static double CLAW_PAN_POSITION_AUTO_PICKUP = 0.225;
  public static double CLAW_PAN_POSITION_AUTO_PICKUP_DIP = 0.135;          ;
  public static double CLAW_PAN_SPEED = 0.025;

  public static double CLAW_ROTATE_SPEED = 0.100;
  public static double CLAW_ROTATE_MAX = 0.8125;
  public static double CLAW_ROTATE_MIN = 0.1825;
  public static double CLAW_ROTATE_POSITION_INIT = 0.0;

  // Extension constants with 117 RPM motor
//  public static int ARM_EXT_INIT = 0;
//  public static int ARM_EXT_DROP_TOP_BASKET = 8085;
//  public static int ARM_EXT_DROP_BOTTOM_BASKET = 2550;
//  public static int ARM_EXT_HANG_TOP_SPECIMEN = 1450;
//  public static int ARM_EXT_HANG_TOP_SPECIMEN_PULL = 140;
//  public static int ARM_EXT_PICKUP_SAMPLES = 2293;
//  public static int ARM_EXT_DRIVE = 0;
//  public static int ARM_EXT_PICKUP_WALL = 0;
//  public static int ARM_EXT_HANG_ROBOT = 4950;
//  public static int ARM_EXT_HANG_ROBOT_PULL = 1652;
//  public static int ARM_EXT_AUTO_HANG = 2320;
//  public static int ARM_EXT_AUTO_HANG_PULL = 350;
//  public static int ARM_EXT_AUTO_PICKUP = 2000;
//  public static int ARM_EXT_AUTO_DROP_OBSERVE = 4600;

  // Extension constants with 312 RPM motor
  public static int ARM_EXT_INIT = 0;
  public static int ARM_EXT_UP_A_LITTLE = 100;
  public static int ARM_EXT_DROP_TOP_BASKET = 2000; //TODO: test and tune
  public static int ARM_EXT_DROP_BOTTOM_BASKET = 1024;
  public static int ARM_EXT_HANG_TOP_SPECIMEN = 628;
  public static int ARM_EXT_HANG_TOP_SPECIMEN_PULL = 28;
  public static int ARM_EXT_PICKUP_SAMPLES = 0;
  public static int ARM_EXT_PICKUP_SAMPLES_EXT = 1661;
  public static int ARM_EXT_DRIVE = 0;
  public static int ARM_EXT_PICKUP_WALL = 28;
  public static int ARM_EXT_HANG_ROBOT = 2100;
  public static int ARM_EXT_HANG_ROBOT_PULL = 200;

  public static int ARM_EXT_AUTO_HANG = 1040; // TODO: 1009(1:21pm) 24Jan->1034->1040 little higher for different grip on spec
  public static int ARM_EXT_AUTO_HANG_PULL = 425;
  public static int ARM_EXT_AUTO_PICKUP = 1152; //todo fine tune
  public static int ARM_EXT_AUTO_DROP_OBSERVE = (int) (4600/2.66); //todo fine tune
  public static int ARM_EXT_AUTO_DROP_TOP_BASKET = 3060;

  public static int ARM_ROT_INIT = 0;
  public static int ARM_ROT_DROP_OFF_SAMPLES = 200;
  public static int ARM_ROT_DROP_OFF_SAMPLES_BOTTOM = 1525;
  public static int ARM_ROT_HANG_TOP_SPECIMEN = 1202;
  public static int ARM_ROT_PICKUP_SAMPLES = 286;
  public static int ARM_ROT_PICKUP_WALL = 297;
  public static int ARM_ROT_AUTO_PICKUP_WALL = 200;
  public static int ARM_ROT_DRIVE = 547;
  public static int ARM_ROT_HANG_ROBOT = 1050;
  public static int ARM_ROT_AUTO_HANG = 1040; //1068; //1160
  public static int ARM_ROT_AUTO_DROP_OFF_SAMPLES = 1560;
  public static int ARM_ROT_AUTO_DRIVE = 1123;
  public static int ARM_ROT_AUTO_PICKUP = 356;

  public static double ARM_ROT_POWER = 0.5;
  public static double ARM_ROT_POWER_FULL = 1.0;
  public static double ARM_EXT_POWER = 1.0;
  public static double ARM_EXT_POWER_AUTO = 0.38;
  public static double DRIVE_TRAIN_SPEED_FAST = 1.0;
  public static double DRIVE_TRAIN_SPEED_SLOW = 1.0 / 3.0;

  public static double LENGTH_CLAW = 7;
  public static double LENGTH_INSPECTION_FRONT = 35;
  public static double LENGTH_INSPECTION_BACK = 0;
  public static double LENGTH_ARM_EXTENDED = 50;
  public static double LENGTH_ARM_NORMAL = 13.375;
  public static double LIMELIGHT_CAMERA_HEIGHT = 12.5;
  public static double LIMELIGHT_CAMERA_ANGLE = 80.0;
  public static double LENGTH_CIRCUMFERENCE_WHEEL = 10.995574287564276;

  public static double CONVERT_DEGREES_TICKS_117RPM = 3.95861111111;
  public static double CONVERT_TICKS_DEGREES_117RPM = 1.0 / CONVERT_DEGREES_TICKS_117RPM;
  public static double CONVERT_DEGREES_TICKS_312RPM = 1.4936111111;
  public static double CONVERT_TICKS_DEGREES_312RPM = 1.0 / CONVERT_DEGREES_TICKS_117RPM;
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
  int slideRotationTargetPosition = ARM_ROT_DRIVE;
  int slideExtensionTargetPosition = ARM_EXT_INIT;

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
    slideExtensionMotorRight = myOpMode.hardwareMap.get(DcMotor.class, "slideExtensionMotorRight");
    slideExtensionMotorLeft = myOpMode.hardwareMap.get(DcMotor.class, "slideExtensionMotorLeft");
    armRotationMotor = myOpMode.hardwareMap.get(DcMotor.class, "armRotationMotor");

    clawGrabServo = myOpMode.hardwareMap.get(Servo.class, "clawGrabServo");
    clawPanServo = myOpMode.hardwareMap.get(Servo.class, "clawPanServo");
    clawRotateServo = myOpMode.hardwareMap.get(Servo.class, "clawRotateServo");

    armRotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    armRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    armRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armRotationMotor.setTargetPosition(ARM_ROT_INIT);
    armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armRotationMotor.setPower(ARM_ROT_POWER_FULL);


    slideExtensionMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
    slideExtensionMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slideExtensionMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slideExtensionMotorRight.setTargetPosition(ARM_EXT_INIT);
    slideExtensionMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    slideExtensionMotorRight.setPower(1);

    slideExtensionMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    slideExtensionMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slideExtensionMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slideExtensionMotorLeft.setTargetPosition(ARM_EXT_INIT);
    slideExtensionMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    slideExtensionMotorLeft.setPower(1);

    clawGrabServo.setPosition(CLAW_GRAB_POSITION_CLOSED);
    clawPanServo.setPosition(CLAW_PAN_TELEOP_INIT);
    clawRotateServo.setPosition(CLAW_ROTATE_POSITION_INIT);

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

  // Set Slide Extension Motor power
  public void setSlideExtensionMotorPower(double power) {
    slideExtensionMotorRight.setPower(power);
    slideExtensionMotorLeft.setPower(power);

  }

  public void setSlideExtMotorTargetPos(int position) {
    slideExtensionMotorRight.setTargetPosition(position);
    slideExtensionMotorLeft.setTargetPosition(position);
  }

  // Get Slide Extension Motor Target position
  public int getSlideExtensionMotorTargetPosition() {
    return slideExtensionMotorRight.getTargetPosition();
  }

  // Get Slide Extension Motor Target position
  public int getSlideExtensionMotorCurrentPosition() {
    return slideExtensionMotorRight.getCurrentPosition();
  }
  // Toggle Finger position
  public void toggleClawGrabPosition() {
    clawGrabPosition = clawGrabServo.getPosition() == CLAW_GRAB_POSITION_CLOSED ? CLAW_GRAB_POSITION_OPEN : CLAW_GRAB_POSITION_CLOSED;
  }

  // Move claw wrist up
  public void clawPanServoUp() {
    clawPanPosition = (clawPanServo.getPosition() + CLAW_PAN_SPEED) > 1 ? 1 : (clawPanServo.getPosition() + CLAW_PAN_SPEED);
  }

  // Move claw wrist down
  public void clawPanServoDown() {
    clawPanPosition = (clawPanServo.getPosition() - CLAW_PAN_SPEED) < 0 ? 0 : (clawPanServo.getPosition() - CLAW_PAN_SPEED);
  }

  // Move claw wrist up
  public void clawRotateServoLeft() {
    clawRotatePosition = (clawRotateServo.getPosition() + CLAW_ROTATE_SPEED) > CLAW_ROTATE_MAX ? CLAW_ROTATE_MAX : (clawRotateServo.getPosition() + CLAW_PAN_SPEED);
  }

  // Move claw wrist down
  public void clawRotateServoRight() {
    clawRotatePosition = (clawRotateServo.getPosition() - CLAW_ROTATE_SPEED) < CLAW_ROTATE_MIN ? CLAW_ROTATE_MIN : (clawRotateServo.getPosition() - CLAW_PAN_SPEED);
  }

  // Set Slide Rotation Motor power
  public void setArmRotationMotorPower(double power) {
    armRotationMotor.setPower(power);
  }

  // Set Slide Rotation Motor position
  public void setArmRotationMotorTargetPosition(int position) {
    armRotationMotor.setTargetPosition(position);
  }

  // Get Slide Rotation Motor Target position
  public int getSlideRotationMotorTargetPosition() {

    return armRotationMotor.getTargetPosition();
  }

  /*// Set Slide Rotation Motor power
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
    checkSoftLimits(convertTicksToDegrees312RPM(slideExtensionMotor.getCurrentPosition()) * Robot.CONVERT_DEGREES_INCHES_SLIDE,
            (slideRotationMotor.getCurrentPosition() - 327) / 14.6697222222);

    slideExtensionTargetPosition = convertDegreesToTicks312RPM((maxExtension - LENGTH_ARM_NORMAL) * CONVERT_INCHES_DEGREES_SLIDE);
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

  // Move claw wrist up
  public void clawRotateServoLeft() {
    clawRotatePosition = (clawRotateServo.getPosition() + CLAW_ROTATE_SPEED) > CLAW_ROTATE_MAX ? CLAW_ROTATE_MAX : (clawRotateServo.getPosition() + CLAW_PAN_SPEED);
  }

  // Move claw wrist down
  public void clawRotateServoRight() {
    clawRotatePosition = (clawRotateServo.getPosition() - CLAW_ROTATE_SPEED) < CLAW_ROTATE_MIN ? CLAW_ROTATE_MIN : (clawRotateServo.getPosition() - CLAW_PAN_SPEED);
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
    clawPanPosition = CLAW_PAN_POSITION_STRAIGHT;
    clawGrabPosition = CLAW_GRAB_POSITION_OPEN;
  }

  public void dropTopBasket() {
    slideRotationMotor.setPower(ARM_ROT_POWER);
    slideRotationTargetPosition = ARM_ROT_DROP_OFF_SAMPLES;
    slideExtensionMotor.setPower(ARM_EXT_POWER);
    slideExtensionTargetPosition = ARM_EXT_DROP_TOP_BASKET;
    clawPanPosition = CLAW_PAN_POSITION_STRAIGHT;
  }

  // Bottom Basket Drop
  public void dropBottomBasket() {
    slideRotationMotor.setPower(ARM_ROT_POWER);
    slideRotationTargetPosition = ARM_ROT_DROP_OFF_SAMPLES_BOTTOM;
    slideExtensionMotor.setPower(ARM_EXT_POWER);
    slideExtensionTargetPosition = ARM_EXT_DROP_BOTTOM_BASKET;
    clawPanPosition = CLAW_PAN_POSITION_STRAIGHT;
  }

  //Wall Pickup
  public void wallPickup () {
    slideRotationMotor.setPower(ARM_ROT_POWER);
    slideRotationTargetPosition = ARM_ROT_PICKUP_WALL;
    slideExtensionMotor.setPower(ARM_EXT_POWER);
    slideExtensionTargetPosition = ARM_EXT_PICKUP_WALL;
    clawPanPosition = CLAW_PAN_POSITION_PICKUP_WALL;
    clawGrabPosition = CLAW_GRAB_POSITION_OPEN;
  }

  // Top Specimen Bar
  public void topSpecimenBar () {
    slideRotationMotor.setPower(ARM_ROT_POWER);
    slideRotationTargetPosition = ARM_ROT_HANG_TOP_SPECIMEN;
    clawPanPosition = CLAW_PAN_POSITION_TOP_SPECIMEN;
    slideExtensionMotor.setPower(ARM_EXT_POWER);
    slideExtensionTargetPosition = ARM_EXT_HANG_TOP_SPECIMEN;
  }

  public void pullExtToHangSpecimen() {
    slideExtensionMotor.setPower(ARM_EXT_POWER);
    slideExtensionTargetPosition = ARM_EXT_HANG_TOP_SPECIMEN_PULL;

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

  public void checkExtentionLimit () {
    // check limit
    checkSoftLimits(convertTicksToDegrees312RPM(slideExtensionMotor.getCurrentPosition()) * Robot.CONVERT_DEGREES_INCHES_SLIDE,
            slideRotationMotor.getCurrentPosition() / 14.6697222222 - 17.6);
    // check to see if the
    int max = convertDegreesToTicks312RPM((maxExtension - LENGTH_ARM_NORMAL) * CONVERT_INCHES_DEGREES_SLIDE);
    if (slideExtensionMotor.getCurrentPosition() > max) {
      slideExtensionTargetPosition = max;
    }
    if (slideExtensionTargetPosition > ARM_EXT_DROP_TOP_BASKET)
      slideExtensionTargetPosition = ARM_EXT_DROP_TOP_BASKET;
    else if (slideExtensionTargetPosition < ARM_EXT_INIT)
      slideExtensionTargetPosition = ARM_EXT_INIT;

  }
  public void moveArmToPosition() {
    myOpMode.telemetry.addData("slideRotationTargetPosition", "start %d", slideRotationTargetPosition);
    myOpMode.telemetry.addData("slideExtensionTargetPosition", "start %d", slideExtensionTargetPosition);
    if (slideExtensionTargetPosition > slideExtensionMotor.getCurrentPosition()) {
      slideRotationMotor.setTargetPosition(slideRotationTargetPosition);
      if (Math.abs(slideRotationTargetPosition - slideRotationMotor.getCurrentPosition()) < 10) {
        checkExtentionLimit();
        slideExtensionMotor.setTargetPosition(slideExtensionTargetPosition);
      }
    } else if (slideExtensionTargetPosition < slideExtensionMotor.getCurrentPosition()) {
      slideExtensionMotor.setTargetPosition(slideExtensionTargetPosition);
      if (Math.abs(slideExtensionTargetPosition - slideExtensionMotor.getCurrentPosition()) < 30) {
        slideRotationMotor.setTargetPosition(slideRotationTargetPosition);
      }
    } else {
      slideRotationMotor.setTargetPosition(slideRotationTargetPosition);
      checkExtentionLimit();
      slideExtensionMotor.setTargetPosition(slideExtensionTargetPosition);
    }
    myOpMode.telemetry.addData("slideRotationTargetPosition", "end %d", slideRotationTargetPosition);
    myOpMode.telemetry.addData("slideExtensionTargetPosition", "end %d", slideExtensionTargetPosition);
  }

  public boolean armReachedTarget() {
    if (Math.abs(slideExtensionTargetPosition - slideExtensionMotor.getCurrentPosition()) < 10
            && Math.abs(slideRotationTargetPosition - slideRotationMotor.getCurrentPosition()) < 10)
      return true;
    else
      return false;
  }
  public void moveArmToDropTop() {
    moveArmToPosition();

  }
  public void moveHandToPosition() {
    clawGrabServo.setPosition(clawGrabPosition);
    clawPanServo.setPosition(clawPanPosition);
    clawRotateServo.setPosition(clawRotatePosition);
  }

  public void handPickUpdip() {
    clawPanPosition = CLAW_PAN_POSITION_PICKUP_DIP;
    clawPanServo.setPosition(clawPanPosition);
  }

  public boolean isFingersOpen() {
    return (clawGrabServo.getPosition() == CLAW_GRAB_POSITION_OPEN);
  }
  public void closeFingers() {
    clawGrabPosition = CLAW_GRAB_POSITION_CLOSED;
    clawGrabServo.setPosition(clawGrabPosition);
  }
  public void openFingers() {
    clawGrabPosition = CLAW_GRAB_POSITION_OPEN;
    clawGrabServo.setPosition(clawGrabPosition);
  }
  public void handStraight() {
    clawPanPosition = CLAW_PAN_POSITION_STRAIGHT;
    clawPanServo.setPosition(clawPanPosition);
  }

  public void handDropTopDip () {
    clawPanPosition = CLAW_PAN_POSITION_DROP_DIP;
    clawPanServo.setPosition(clawPanPosition);

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

  public int convertDegreesToTicks312RPM(double degrees) {
    return (int) Math.round(degrees * CONVERT_DEGREES_TICKS_312RPM);
  }

  public double convertTicksToDegrees312RPM(int ticks) {
    return ticks * CONVERT_TICKS_DEGREES_312RPM;
  }
  public void getReadyToHangRobot() {
    slideRotationMotor.setPower(ARM_ROT_POWER);
    slideRotationTargetPosition = ARM_ROT_HANG_ROBOT;
    slideExtensionMotor.setPower(ARM_EXT_POWER);
    slideExtensionTargetPosition = ARM_EXT_HANG_ROBOT;
    clawPanPosition = CLAW_PAN_POSITION_HANG_ROBOT;
  }

  public void HangRobot() {
    slideExtensionMotor.setPower(ARM_EXT_POWER);
    slideExtensionTargetPosition = ARM_EXT_HANG_ROBOT_PULL;
  }

  public boolean isArmPickup() {
    return (Math.abs(slideRotationMotor.getCurrentPosition() - ARM_ROT_PICKUP_SAMPLES) < 130) &&
            ((slideExtensionMotor.getCurrentPosition() > ARM_EXT_PICKUP_SAMPLES) &&
                    (slideExtensionMotor.getCurrentPosition() < (ARM_EXT_PICKUP_SAMPLES_EXT + 50))) &&
            (Math.abs(clawPanServo.getPosition() - CLAW_PAN_POSITION_STRAIGHT) <= 0.1);
  }

  public boolean isArmTopDropReady() {
    return (Math.abs(slideRotationMotor.getCurrentPosition() - ARM_ROT_DROP_OFF_SAMPLES) < 100) &&
            (Math.abs(slideExtensionMotor.getCurrentPosition() - ARM_EXT_DROP_TOP_BASKET) < 500) &&
            (Math.abs(clawPanServo.getPosition() - CLAW_PAN_POSITION_DROP_DIP) <= 0.1);
  }

  public boolean isArmBottomDropReady() {
    return (Math.abs(slideRotationMotor.getCurrentPosition() - ARM_ROT_DROP_OFF_SAMPLES_BOTTOM) < 100) &&
            (Math.abs(slideExtensionMotor.getCurrentPosition() - ARM_EXT_DROP_BOTTOM_BASKET) < 500) &&
            (Math.abs(clawPanServo.getPosition() - CLAW_PAN_POSITION_DROP_DIP) <= 0.1);
  }

  public void setRotationTargetForPickUp() {
    double a = (105.0/1661.0);
    double rotationOffSet = a * slideExtensionMotor.getCurrentPosition();
    slideRotationTargetPosition = (int)(rotationOffSet) + ARM_ROT_PICKUP_SAMPLES;
  }*/

}

