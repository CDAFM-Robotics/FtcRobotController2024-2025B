package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Zero Servo", group = "utility")
public class ZeroServoOpMode extends LinearOpMode {
  Servo clawGrabServo = null;
  @Override
  public void runOpMode() throws InterruptedException {
    clawGrabServo = hardwareMap.get(Servo.class, "clawGrabServo");

    clawGrabServo.setPosition(0);

    waitForStart();
  }
}
