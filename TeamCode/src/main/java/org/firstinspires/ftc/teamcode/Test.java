package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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

    CRServo indexServo = null;
    Servo flickerServo = null;
    ColorSensor loc4 = null;
    ColorSensor loc2 = null;
    ColorSensor loc3 = null;
    RevTouchSensor limitSwitch = null;

    ColorSensor[] sensors = {loc4, loc3, loc2};

    DcMotor intake;
    DcMotorEx shooter;

    double indexServoPos = .02;
    double flickerServoPos = 0.45;

    @Override
    public void runOpMode() {
        indexServo = hardwareMap.get(CRServo.class, "index");
        flickerServo = hardwareMap.get(Servo.class, "flicker");
        limitSwitch = hardwareMap.get(RevTouchSensor.class, "limiter");
        loc4 = hardwareMap.get(ColorSensor.class, "location4");
        loc2 = hardwareMap.get(ColorSensor.class, "location3");
        loc3 = hardwareMap.get(ColorSensor.class, "location2");


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
                indexServo.setPower(0.5);
            } else if (gamepad1.leftBumperWasPressed()) {
                indexServo.setPower(-0.5);
            }
        } else {
            if (gamepad1.rightBumperWasPressed()) {
                indexServo.setPower(0.1);
            } else if (gamepad1.leftBumperWasPressed()) {
                indexServo.setPower(-0.1);
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


            flickerServo.setPosition(flickerServoPos);

            telemetry.addData("velocity", shooter.getVelocity());
            telemetry.addData();
            telemetry.addData("Index Servo Position", indexServoPos);
            telemetry.addData("Flicker` Servo Position", flickerServoPos);
            telemetry.addData("Color Sensors", getColorReadings());

            telemetry.addData("\nPose storage", "\nx: " + PoseStorage.currentPose.position.x + "\ny: " + PoseStorage.currentPose.position.y);

            telemetry.update();
        }
    }


    private String getColorReadings() {
        String output = "";
        float[] hsvValues = {0,0,0};
        Color.RGBToHSV(loc4.red() * 8, loc4.green() * 8, loc4.blue() * 8, hsvValues);
        output += "Sensor 4: \nh:" + hsvValues[0] + "\ns:" + hsvValues[1] + "\nv:" + hsvValues[2] + "\nAlpha: " + loc4.alpha() + ".\n";

        Color.RGBToHSV(loc3.red() * 8, loc3.green() * 8, loc3.blue() * 8, hsvValues);
        output += "\nSensor 3: \nh:" + hsvValues[0] + "\ns:" + hsvValues[1] + "\nv:" + hsvValues[2] + "\nAlpha: " + loc4.alpha() + ".\n";

        Color.RGBToHSV(loc2.red() * 8, loc2.green() * 8, loc2.blue() * 8, hsvValues);
        output += "\nSensor 2: \nh:" + hsvValues[0] + "\ns:" + hsvValues[1] + "\nv:" + hsvValues[2] + "\nAlpha: " + loc4.alpha() + ".\n";

        return output;
    }


    //..431, .465, .506, .543, .576, .613

}
