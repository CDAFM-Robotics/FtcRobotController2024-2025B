package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {

  // Motors and devices
  DcMotor leftFrontMotor = null;
  DcMotor rightFrontMotor = null;
  DcMotor leftBackMotor = null;
  DcMotor rightBackMotor = null;

  DcMotor slideExtensionMotor = null;
  DcMotor slideRotationMotor = null;
  DcMotor linearActuatorMotor = null;

  Servo clawGrabServo = null;
  Servo clawRotateServo = null;

  LinearOpMode myOpMode;


  public Robot(LinearOpMode opMode) {
    myOpMode = opMode;
  }

  public void initializeDevices() {

    myOpMode.telemetry.addData("Status", "Initializing");
    myOpMode.telemetry.update();

    leftFrontMotor = myOpMode.hardwareMap.get(DcMotor.class, "leftFrontMotor");
    rightFrontMotor = myOpMode.hardwareMap.get(DcMotor.class, "rightFrontMotor");
    leftBackMotor = myOpMode.hardwareMap.get(DcMotor.class, "leftBackMotor");
    rightBackMotor = myOpMode.hardwareMap.get(DcMotor.class, "rightBackMotor");

    slideExtensionMotor = myOpMode.hardwareMap.get(DcMotor.class, "slideExtensionMotor");
    slideRotationMotor = myOpMode.hardwareMap.get(DcMotor.class, "slideRotationMotor");
    linearActuatorMotor = myOpMode.hardwareMap.get(DcMotor.class, "linearActuatorMotor");

    clawGrabServo = myOpMode.hardwareMap.get(Servo.class, "clawGrabServo");
    clawRotateServo = myOpMode.hardwareMap.get(Servo.class, "clawRotateServo");
  }

}
