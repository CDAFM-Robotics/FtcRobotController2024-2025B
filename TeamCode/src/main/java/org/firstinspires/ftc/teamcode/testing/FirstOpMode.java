package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "First Op Mode", group = "zTesting")

//@Disabled
public class FirstOpMode extends LinearOpMode {

  private ElapsedTime runtime = new ElapsedTime();
  private DcMotor frontLeftMotor = null;
  private DcMotor frontRightMotor = null;
  private DcMotor backLeftMotor = null;
  private DcMotor backRightMotor = null;

  Gamepad currentGamepad1 = new Gamepad();
  Gamepad previousGamepad1 = new Gamepad();
  Gamepad currentGamepad2 = new Gamepad();
  Gamepad previousGamepad2 = new Gamepad();

  double lStickY;
  double lStickX;
  double rStickX;

  double rotX;
  double rotY;
  double denominator;
  double frontLeftPower;
  double frontRightPower;
  double backLeftPower;
  double backRightPower;

  @Override
  public void runOpMode() {

    telemetry.addData("Status", "Initializing");
    telemetry.addLine("line");
    telemetry.update();

    frontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
    frontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
    backLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
    backRightMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");

    telemetry.addData("Status", "Initialized");
    telemetry.update();

    waitForStart();

    while (opModeIsActive()) {
      previousGamepad1.copy(currentGamepad1);
      previousGamepad2.copy(currentGamepad2);
      currentGamepad1.copy(gamepad1);
      currentGamepad2.copy(gamepad2);

      lStickX = currentGamepad1.left_stick_x;
      lStickY = currentGamepad1.left_stick_y;
      rStickX = currentGamepad1.right_stick_x;

      setMotorPowers(Math.pow(lStickX, 3), Math.pow(lStickY, 3), Math.pow(rStickX, 3), 0);

      telemetry.addData("Status", "Running");
      telemetry.addData("Stick Powers", "Left Stick X: %.2f, Left Stick Y: %.2f, Right Stick X: %.2f", lStickX, lStickY, rStickX);
      telemetry.addData("Motor Powers", "Front Left: %.2f, Front Right: %.2f, Back Left: %.2f, Back Right: %.2f");
    }
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
  }
}
