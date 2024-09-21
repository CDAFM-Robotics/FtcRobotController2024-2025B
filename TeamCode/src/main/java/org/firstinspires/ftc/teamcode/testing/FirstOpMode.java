package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

  @Override
  public void runOpMode() {

    telemetry.addData("Status", "Initializing");
    telemetry.update();

    frontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
    frontRightMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
    backLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
    backRightMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");

    telemetry.addData("Status", "Initialized");
    telemetry.update();

    waitForStart();

    while (opModeIsActive()) {

    }
  }
  public void setMotorPowers(double x, double y, double rx, double heading) {
    double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
    double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

    // put strafing factors here
    rotX = rotX * 1;
    rotY = rotY * 1;

    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

    double frontLeftPower = (rotY + rotX + rx)/denominator;
    double backLeftPower = (rotY - rotX + rx)/denominator;
    double frontRightPower = (rotY - rotX - rx)/denominator;
    double backRightPower = (rotY + rotX - rx)/denominator;

    frontLeftMotor.setPower(frontLeftPower);
    backLeftMotor.setPower(backLeftPower);
    frontRightMotor.setPower(frontRightPower);
    backRightMotor.setPower(backRightPower);
  }
}
