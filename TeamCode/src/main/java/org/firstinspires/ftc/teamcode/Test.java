package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RoadRunnerAutos.PoseStorage;
import org.firstinspires.ftc.teamcode.RoadRunnerAutos.Shooter;
@TeleOp(name="TEST", group="Linear OpMode")
public class Test extends LinearOpMode {

    Servo indexServo = null;
    Servo flickerServo = null;

    DcMotor intake;
    DcMotorEx shooter;

    double indexServoPos = .02;
    double flickerServoPos = 0.45;

    @Override
    public void runOpMode() {
        indexServo = hardwareMap.get(Servo.class, "index");
        flickerServo = hardwareMap.get(Servo.class, "flicker");

        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            intake.setPower(gamepad1.right_stick_y);


            if (gamepad1.a) {
                if (gamepad1.rightBumperWasPressed()) {
                    indexServoPos += 0.1;
                } else if (gamepad1.leftBumperWasPressed()) {
                    indexServoPos -= 0.1;
                }
            } else {
                if (gamepad1.rightBumperWasPressed()) {
                    indexServoPos += 0.001;
                } else if (gamepad1.leftBumperWasPressed()) {
                    indexServoPos -= 0.001;
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

            telemetry.addData("velocity", shooter.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("Index Servo Position", indexServoPos);
            telemetry.addData("Flicker` Servo Position", flickerServoPos);

            telemetry.addData("\nPose storage", "\nx: " + PoseStorage.currentPose.position.x + "\ny: " + PoseStorage.currentPose.position.y);

            telemetry.update();
        }
    }
}
