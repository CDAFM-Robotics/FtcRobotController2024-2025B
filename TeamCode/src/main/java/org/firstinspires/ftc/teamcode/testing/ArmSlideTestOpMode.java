package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Arm Slide Test", group = "zTesting")
@Disabled
public class ArmSlideTestOpMode extends LinearOpMode {
  DcMotor slideExtMotor = null;
  DcMotor slideRotMotor = null;
  @Override
  public void runOpMode() {
    slideExtMotor = hardwareMap.get(DcMotor.class, "slideExtensionMotor");
    slideRotMotor = hardwareMap.get(DcMotor.class, "slideRotationMotor");

    waitForStart();

    while (opModeIsActive()) {
      slideExtMotor.setPower(gamepad2.right_stick_y);
      slideRotMotor.setPower(gamepad2.left_stick_y);
    }
  }
}
