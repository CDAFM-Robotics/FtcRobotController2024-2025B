package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ARMCONTROL", group = "testing")
public class newArmControlTestOpMode extends LinearOpMode{
    Robot robot = new Robot(this);
    DcMotor linearActuatorRight = null;
    DcMotor linearActuatorLeft = null;
    DcMotor slideRotationMotor = null;
    DcMotor slideExtensionMotor = null;

    Servo clawGrabServo = null;

    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void runOpMode() {

        linearActuatorRight = hardwareMap.get(DcMotor.class, "slideExtensionMotorRight");
        linearActuatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearActuatorRight.setTargetPosition(0);
        linearActuatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        linearActuatorLeft = hardwareMap.get(DcMotor.class, "slideExtensionMotorLeft");
        linearActuatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearActuatorLeft.setTargetPosition(0);
        linearActuatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideRotationMotor = hardwareMap.get(DcMotor.class, "slideRotationMotor");
        slideRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRotationMotor.setTargetPosition(0);
        slideRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideExtensionMotor = hardwareMap.get(DcMotor.class, "slideUpMotor");
        slideExtensionMotor.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        slideExtensionMotor.setTargetPosition(0);
        slideExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideExtensionMotor.setMode((DcMotor.RunMode.RUN_TO_POSITION));

        clawGrabServo = hardwareMap.get(Servo.class, "clawGrabServo");
        clawGrabServo.setPosition(Robot.CLAW_GRAB_POSITION_OPEN);

        boolean open = true;
        waitForStart();


        while(opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            //linear actuators
//            if(currentGamepad1.a && !previousGamepad1.a && linearActuatorRight.getCurrentPosition() < 12400){
//
//                linearActuatorRight.setPower(1);
//                linearActuatorRight.setTargetPosition(linearActuatorRight.getTargetPosition() + 500);
//            }
//            if(currentGamepad1.a && !previousGamepad1.a && linearActuatorLeft.getCurrentPosition() < 12400){
//
//                linearActuatorLeft.setPower(-1);
//                linearActuatorLeft.setTargetPosition(linearActuatorLeft.getTargetPosition() - 500);
//            }
//
//            if(currentGamepad1.x && !previousGamepad1.x && linearActuatorRight.getCurrentPosition() > 10){
//
//                linearActuatorRight.setTargetPosition(linearActuatorRight.getTargetPosition() - 500);
//                linearActuatorRight.setPower(-1);
//
//
//            }
//            if(currentGamepad1.x && !previousGamepad1.x && linearActuatorLeft.getCurrentPosition() > 10){
//                linearActuatorLeft.setTargetPosition(linearActuatorLeft.getTargetPosition() + 500);
//                linearActuatorLeft.setPower(1);
//
//            }

            //claw servos
//            if(currentGamepad1.right_bumper && !previousGamepad1.right_bumper && clawGrabServo.getPosition() != 0){
//                clawGrabServo.setPosition(0);
//                open = false;
//            } else if (clawGrabServo.getPosition() != 0.625){
//                clawGrabServo.setPosition(0.625);
//                open = true;
//            }
            if(currentGamepad1.right_bumper && !previousGamepad1.right_bumper ){
                if (clawGrabServo.getPosition() == Robot.CLAW_GRAB_POSITION_OPEN) {
                    clawGrabServo.setPosition(Robot.CLAW_GRAB_POSITION_CLOSED);

                }
                else {
                    clawGrabServo.setPosition(Robot.CLAW_GRAB_POSITION_OPEN);
                }
            }



            //arm rotation motor
            if(currentGamepad1.y && !previousGamepad1.y){
                slideRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideRotationMotor.setPower(0.5);
                slideRotationMotor.setTargetPosition(10);
            }
            if(currentGamepad1.b && !previousGamepad1.b){
                slideRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideRotationMotor.setPower(0.5);
                slideRotationMotor.setTargetPosition(-10);
            }

            //slide extension motor
            if(currentGamepad1.dpad_up && currentGamepad1.dpad_up && slideExtensionMotor.getTargetPosition() != -27000){
                slideExtensionMotor.setPower(3);
                slideExtensionMotor.setTargetPosition(-27000);
            }
            if(currentGamepad1.dpad_down && !previousGamepad1.dpad_down && slideExtensionMotor.getTargetPosition() != 0){
                slideExtensionMotor.setPower(3);
                slideExtensionMotor.setTargetPosition(0);
            }

            telemetry.addData("slide rotation motor current position", slideRotationMotor.getCurrentPosition());
            telemetry.addData("slide rotation motor target position", slideRotationMotor.getTargetPosition());
            telemetry.addData("slide extension motor current position", slideExtensionMotor.getCurrentPosition() );
            telemetry.addData("slide extension motor target position", slideExtensionMotor.getTargetPosition() );
            telemetry.addData("claw grab servo", clawGrabServo.getPosition());
            telemetry.update();
        }
    }
}
