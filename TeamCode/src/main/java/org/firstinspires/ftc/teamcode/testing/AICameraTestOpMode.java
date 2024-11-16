package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.Robot;

@TeleOp (name = "AICAMERA", group = "testing")
public class AICameraTestOpMode extends LinearOpMode {

    Robot robot = new Robot(this);



    private Limelight3A limelight;
    private IMU imu;



    @Override
    public void runOpMode(){
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );
        waitForStart();

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);


        limelight.start();

        while(opModeIsActive()){
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));

            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
            LLResult result = limelight.getLatestResult();

            telemetry.addData("pipeline", result.getPipelineIndex());

            if (result != null){
                if(result.isValid()){
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("Botpose", botpose.toString());
                    telemetry.addData("target x", result.getTx());
                    telemetry.addData("target y", result.getTy());
                    telemetry.addData("target Area", result.getTa());
                }
            }else{
                telemetry.addData("Limelight ", "no targets");
            }

            if(robot.currentGamepad2.a && !robot.previousGamepad2.a){

                int currentIndex = result.getPipelineIndex();
                telemetry.addData("test thing", "yea we pressed it");
                limelight.pipelineSwitch(currentIndex += 1);
            }

            telemetry.update();
        }
    }
}
