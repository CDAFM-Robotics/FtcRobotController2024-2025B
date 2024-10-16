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
  public Servo clawRotateServo = null;

  LinearOpMode myOpMode;



  // Constants

  public double CLAW_GRAB_POSITION_CLOSED = 0.175;
  public double CLAW_GRAB_POSITION_OPEN = 0.0;

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
    clawRotateServo = myOpMode.hardwareMap.get(Servo.class, "clawRotateServo");

    frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
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

  // TODO: Make Trajectories in robot class or other class

}
