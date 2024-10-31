package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {

  // Motors and devices

  public DcMotor frontLeftMotor = null;
  public DcMotor frontRightMotor = null;
  public DcMotor backLeftMotor = null;
  public DcMotor backRightMotor = null;

  public DcMotor slideExtensionMotor = null;
  public DcMotor slideRotationMotor = null;

  public DcMotor linearActuatorLeftMotor = null;
  public DcMotor linearActuatorRightMotor = null;

  public Servo clawGrabServo = null;
  public Servo clawPanServo = null;

  LinearOpMode myOpMode;



  // Constants

  public static double CLAW_GRAB_POSITION_CLOSED = 0.175;
  public static double CLAW_GRAB_POSITION_OPEN = 0.0;
  public static double CLAW_PAN_POSITION_DROP = 0.0;
  public static double CLAW_PAN_POSITION_PICKUP = 0.2;
  public static int ARM_EXT_INIT = 0;
  public static int ARM_EXT_DROP_TOP_BASKET = 3163;
  public static int ARM_EXT_DROP_BOTTOM_BASKET = 922;
  public static int ARM_EXT_DROP_TOP_SPECIMEN = 922;
  public static int ARM_EXT_PICKUP_SAMPLES = 0;
  public static int ARM_ROT_INIT = 0;
  public static int ARM_ROT_DROP_OFF_SAMPLES = 1475;
  public static int ARM_ROT_DROP_TOP_SPECIMEN = 1475;
  public static int ARM_ROT_PICKUP_SAMPLES = 10;

  public static double LENGTH_CLAW = 9;
  public static double LENGTH_INSPECTION_FRONT = 30;
  public static double LENGTH_INSPECTION_BACK = 10;
  public static double LENGTH_ARM_EXTENDED = 53.54331;
  public static double LENGTH_ARM_NORMAL = 13.375;

  public static double CONVERT_DEGREES_TICKS_312RPM = 1.4936111111;
  public static double CONVERT_TICKS_DEGREES_312RPM = 1.0/CONVERT_DEGREES_TICKS_312RPM;
  public static double CONVERT_DEGREES_INCHES_SLIDE = 0.013177365175032795;
  public static double CONVERT_INCHES_DEGREES_SLIDE = 1.0/CONVERT_DEGREES_INCHES_SLIDE;


  // Variables for Functions

  public Gamepad currentGamepad1 = new Gamepad();
  public Gamepad previousGamepad1 = new Gamepad();
  public Gamepad currentGamepad2 = new Gamepad();
  public Gamepad previousGamepad2 = new Gamepad();

  double rotX;
  double rotY;
  double denominator;
  double frontLeftPower;
  double frontRightPower;
  double backLeftPower;
  double backRightPower;

  double maxExtension;
  int slideExtensionTargetPosition = 0;
  int slideRotationTargetPosition = 0;

  boolean prevExtending = false;
  boolean prevRotating = false;



  public Robot(LinearOpMode opMode) {
    myOpMode = opMode;
  }

  public void initializeDevices() {
    frontLeftMotor = myOpMode.hardwareMap.get(DcMotor.class, "frontLeftMotor");
    frontRightMotor= myOpMode.hardwareMap.get(DcMotor.class, "frontRightMotor");
    backLeftMotor = myOpMode.hardwareMap.get(DcMotor.class, "backLeftMotor");
    backRightMotor = myOpMode.hardwareMap.get(DcMotor.class, "backRightMotor");

    slideExtensionMotor = myOpMode.hardwareMap.get(DcMotor.class, "slideExtensionMotor");
    slideRotationMotor = myOpMode.hardwareMap.get(DcMotor.class, "slideRotationMotor");

    linearActuatorLeftMotor = myOpMode.hardwareMap.get(DcMotor.class, "linearActuatorLeftMotor");
    linearActuatorRightMotor = myOpMode.hardwareMap.get(DcMotor.class, "linearActuatorRightMotor");

    clawGrabServo = myOpMode.hardwareMap.get(Servo.class, "clawGrabServo");
    clawPanServo = myOpMode.hardwareMap.get(Servo.class, "clawRotateServo");

    frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    slideRotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    slideRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slideRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slideRotationMotor.setTargetPosition(ARM_ROT_INIT);
    slideRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    slideRotationMotor.setPower(1);


    slideExtensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    slideExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slideExtensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    slideExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slideExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    slideExtensionMotor.setTargetPosition(ARM_EXT_INIT);
    slideExtensionMotor.setPower(1);

    linearActuatorLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    linearActuatorRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    clawPanServo.setPosition(0);
    clawGrabServo.setPosition(0);
  }

  public void setMotorPowers(double x, double y, double rx, double heading) {
    rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
    rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

    // put strafing factors here
    rotX = rotX * 1;
    rotY = rotY * 1;

    denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

    frontLeftPower = (rotY + rotX + rx)/denominator;
    backLeftPower = (rotY - rotX + rx)/denominator;
    frontRightPower = (rotY - rotX - rx)/denominator;
    backRightPower = (rotY + rotX - rx)/denominator;

    frontLeftMotor.setPower(frontLeftPower);
    backLeftMotor.setPower(backLeftPower);
    frontRightMotor.setPower(frontRightPower);
    backRightMotor.setPower(backRightPower);

    myOpMode.telemetry.addData("Motor Powers", "Front Left: %.2f, Front Right: %.2f, Back Left: %.2f, Back Right: %.2f", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
  }

  public void updateGamepads() {
    previousGamepad1.copy(currentGamepad1);
    currentGamepad1.copy(myOpMode.gamepad1);

    previousGamepad2.copy(currentGamepad2);
    currentGamepad2.copy(myOpMode.gamepad2);
  }

  // Set claw Grab Servo position
  public void setClawGrabServoPosition(double grabPosition)
  {
    clawGrabServo.setPosition(grabPosition);
  }

  // Set claw Pan Servo position
  public void setClawPanServoPosition(double panPosition)
  {
    clawPanServo.setPosition(panPosition);
  }

  public void setArmPosition(double armExtension, double armRotation, double extensionPower, double rotationPower) {
    if (extensionPower > 0) {
      slideExtensionMotor.setPower(extensionPower * 0.5);
      slideExtensionTargetPosition = slideExtensionMotor.getCurrentPosition() - 1000;
      prevExtending = true;
    }
    else if (extensionPower < 0) {
      slideExtensionMotor.setPower(extensionPower * 0.5);
      slideExtensionTargetPosition = slideExtensionMotor.getCurrentPosition() + 1000;
      prevExtending = true;
    }
    else {
      if (prevExtending) {
        slideExtensionTargetPosition = slideExtensionMotor.getCurrentPosition();
        prevExtending = false;
      }
      slideExtensionMotor.setPower(0.5);
    }

    checkSoftLimits(armExtension, armRotation);

    if (rotationPower > 0) {
      slideRotationMotor.setPower(rotationPower * 0.5);
      slideRotationTargetPosition = ARM_ROT_DROP_OFF_SAMPLES;
      prevRotating = true;
    }
    else if (rotationPower < 0) {
      slideRotationMotor.setPower(rotationPower * 0.5);
      slideRotationTargetPosition = ARM_ROT_PICKUP_SAMPLES;
      prevRotating = true;
    }
    else {
      if (prevRotating) {
        slideRotationTargetPosition = slideRotationMotor.getCurrentPosition();
        prevRotating = false;
      }
      slideRotationMotor.setPower(0.5);
    }

    setMacros();

    myOpMode.telemetry.addData("Arm Rotation Target Position", slideRotationTargetPosition);
    myOpMode.telemetry.addData("Arm Extension Target Position", slideExtensionTargetPosition);

    slideRotationMotor.setTargetPosition(slideRotationTargetPosition);
    slideExtensionMotor.setTargetPosition(slideExtensionTargetPosition);

  }

  public void checkSoftLimits(double armExtension, double armRotation) {
    if (armRotation <= 90) {
      maxExtension = Math.min((LENGTH_INSPECTION_FRONT - LENGTH_CLAW) / Math.cos(Math.toRadians(armRotation)), LENGTH_ARM_EXTENDED);
    }
    else {
      maxExtension = Math.min((LENGTH_INSPECTION_BACK - LENGTH_CLAW) / Math.cos(Math.toRadians(180 - armRotation)), LENGTH_ARM_EXTENDED);
    }
    if (armExtension + LENGTH_ARM_NORMAL > maxExtension) {
      slideExtensionTargetPosition = convertDegreesToTicks312RPM((maxExtension - LENGTH_ARM_NORMAL) * CONVERT_INCHES_DEGREES_SLIDE);

    }
    myOpMode.telemetry.addData("Arm Rotation", armRotation);
    myOpMode.telemetry.addData("Max Extension Soft Limit", maxExtension);
  }

  public int convertDegreesToTicks312RPM(double degrees) {
    return (int) Math.round(degrees * CONVERT_DEGREES_TICKS_312RPM);
  }

  public double convertTicksToDegrees312RPM(int ticks) {
    return ticks * CONVERT_TICKS_DEGREES_312RPM;
  }

  public void setMacros() {
    // Macros
    // Bottom Basket Drop
    if (currentGamepad2.a && !previousGamepad2.a) {

      slideRotationMotor.setPower(1);
      slideRotationTargetPosition = ARM_ROT_DROP_OFF_SAMPLES;
      setClawPanServoPosition(CLAW_PAN_POSITION_DROP);
      slideExtensionMotor.setPower(1);
      slideExtensionTargetPosition = ARM_EXT_DROP_BOTTOM_BASKET;
    }
    //Top Basket Drop
    if (currentGamepad2.y && !previousGamepad2.y) {

      slideRotationMotor.setPower(1);
      slideRotationTargetPosition = ARM_ROT_DROP_OFF_SAMPLES;
      setClawPanServoPosition(CLAW_PAN_POSITION_DROP);
      slideExtensionMotor.setPower(1);
      slideExtensionTargetPosition = ARM_EXT_DROP_TOP_BASKET;
    }
    //Pickup
    if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {

      slideRotationMotor.setPower(1);
      slideRotationTargetPosition = ARM_ROT_PICKUP_SAMPLES;
      setClawPanServoPosition(CLAW_PAN_POSITION_PICKUP);
      slideExtensionMotor.setPower(1);
      slideExtensionTargetPosition = ARM_EXT_PICKUP_SAMPLES;
    }
    // Top Specimen Bar
    if (currentGamepad2.x && !previousGamepad2.x) {

      slideRotationMotor.setPower(1);
      slideRotationTargetPosition = ARM_ROT_PICKUP_SAMPLES;
      setClawPanServoPosition(CLAW_PAN_POSITION_PICKUP);
      slideExtensionMotor.setPower(1);
      slideExtensionTargetPosition = ARM_EXT_PICKUP_SAMPLES;
    }
    // Drive Position (key is b)

    // Bottom Specimen Bar (key is start)
  }

  // TODO: Make Trajectories in robot class or other class

}
