package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.Robot;

public class InitialZeroPosition extends LinearOpMode {
    Robot robot = new Robot(this);
    /* This is to call the motors and servos we will talk to */
    public Servo clawGrabServo = null;
    public Servo clawPanServo = null;
    public DcMotor slideExtensionMotor = null;
    public DcMotor slideRotationMotor = null;

    /* These are our variables we will tell the devices to be set to*/

    float slideRotationMotorPosition = 0;
    float slideExtensionMotorPosition = 0;
    float clawPanServoPosition = 0;
    float clawGrabServoPosition = 0;

// Read initial position of the extension motor encoder and adjust position value so arm can go there

    @Override
    public void runOpMode() {
        //setting the variables actual value


        //if (condition){

        /*robot.slideExtensionMotor.setPower(slideExtensionMotorPosition);
        robot.slideRotationMotor.setPower(slideRotationMotorPosition);
        robot.clawPanServo.setPosition(clawPanServoPosition);
        robot.clawGrabServo.setPosition(clawGrabServoPosition);*/

    /*slideExtensionMotor.setPower(slideExtensionMotorPosition);
    slideRotationMotor.setPower(slideRotationMotorPosition);
    clawPanServo.setPosition(clawPanServoPosition);
    clawGrabServo.setPosition(clawGrabServoPosition);*/
    }



}

