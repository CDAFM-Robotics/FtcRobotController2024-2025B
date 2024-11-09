package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


    public class TestServo extends LinearOpMode {

        static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
        static final int    CYCLE_MS    =   50;     // period of each cycle
        static final double MAX_POS     =  1.0;     // Maximum rotational position
        static final double MIN_POS     =  0.0;     // Minimum rotational position

        // Define class members
        Servo servo;
        double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
        boolean rampUp = true;


        @Override
        public void runOpMode() {

            // Connect to servo (Assume Robot Left Hand)
            // Change the text in quotes to match any servo name on your robot.
            servo = hardwareMap.get(Servo.class, "clawGrabServo");

            // Wait for the start button
            telemetry.addData(">", "Press Start to scan Servo." );
            telemetry.update();
            waitForStart();


            // Scan servo till stop pressed.
            while(opModeIsActive()){


                servo.setPosition(Math.pow(2,32));
                sleep(CYCLE_MS);
                idle();
            }

            // Signal done;
            telemetry.addData(">", "Done");
            telemetry.update();
        }
    }

