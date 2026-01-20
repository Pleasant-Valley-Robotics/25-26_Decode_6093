package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TEST", group="Linear OpMode")
public class Test extends LinearOpMode {

    Servo indexServo = null;
    Servo flickerServo = null;

    double indexServoPos = .43;
    double flickerServoPos = 0.45;

    @Override
    public void runOpMode() {
        indexServo = hardwareMap.get(Servo.class, "index");
        flickerServo = hardwareMap.get(Servo.class, "flicker");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

        if (gamepad1.a) {
            if (gamepad1.rightBumperWasPressed()) {
                indexServoPos += 0.05;
            } else if (gamepad1.leftBumperWasPressed()) {
                indexServoPos -= 0.05;
            }
        } else {
            if (gamepad1.rightBumperWasPressed()) {
                indexServoPos += 0.01;
            } else if (gamepad1.leftBumperWasPressed()) {
                indexServoPos -= 0.01;
            }
        }


            if (gamepad2.a) {
                if (gamepad2.rightBumperWasPressed()) {
                    flickerServoPos += 0.01;
                } else if (gamepad2.leftBumperWasPressed()) {
                    flickerServoPos -= 0.01;
                }
            } else {
                if (gamepad2.rightBumperWasPressed()) {
                    flickerServoPos += 0.005;
                } else if (gamepad2.leftBumperWasPressed()) {
                    flickerServoPos -= 0.005;
                }
            }


            indexServo.setPosition(indexServoPos);
            flickerServo.setPosition(flickerServoPos);

            telemetry.addData("Index Servo Position", indexServoPos);
            telemetry.addData("Flicker` Servo Position", flickerServoPos);

            telemetry.update();
        }
    }

}
