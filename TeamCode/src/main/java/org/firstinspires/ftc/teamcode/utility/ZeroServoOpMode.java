package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.Robot;

@TeleOp(name = "Test Servo", group = "utility")
public class ZeroServoOpMode extends LinearOpMode {
  Servo servo = null;
  Robot robot = new Robot(this);
  double servoPosition = 0;
  @Override
  public void runOpMode() throws InterruptedException {
    servo = hardwareMap.get(Servo.class, "clawGrabServo");

    servo.setPosition(0);

    waitForStart();

    while (opModeIsActive()) {
      robot.updateGamepads();

      if (robot.currentGamepad2.a && !robot.previousGamepad2.a) {
        servoPosition += 0.025;
        servo.setPosition(servoPosition);
      }
      if (robot.currentGamepad2.b && !robot.previousGamepad2.b) {
        servoPosition -= 0.025;
        servo.setPosition(servoPosition);
      }
      telemetry.addData("Position", servoPosition);
      telemetry.update();
    }
  }
}
