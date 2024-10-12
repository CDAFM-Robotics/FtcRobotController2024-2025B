package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.Robot;

@TeleOp(name = "Hand Control Test", group = "Testing")
public class HandControlTestOpMode extends LinearOpMode {
  Robot robot = new Robot(this);

  Servo clawGrabServo = null;

  Gamepad currentGamepad1;
  Gamepad prevGamepad1;

  double clawClosePosition = 0;

  @Override
  public void runOpMode() {
    clawGrabServo = hardwareMap.get(Servo.class, "clawGrabServo");
    clawGrabServo.setPosition(0.15);
    waitForStart();
    while (opModeIsActive()) {
      prevGamepad1 = currentGamepad1;
      currentGamepad1 = gamepad1;
      if (currentGamepad1.right_bumper && !prevGamepad1.right_bumper) {
        if (clawClosePosition == 0) {
          clawClosePosition = 0.2;
        }
        else {
          clawClosePosition = 0;
        }
      }
      clawGrabServo.setPosition(clawClosePosition);
    }
  }
}
