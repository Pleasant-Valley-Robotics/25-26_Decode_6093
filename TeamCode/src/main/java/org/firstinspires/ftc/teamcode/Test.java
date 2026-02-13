package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

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
    ColorSensor loc4 = null;
    ColorSensor loc2 = null;
    ColorSensor loc3 = null;
    ColorSensor loc1;

    ColorSensor[] sensors = {loc4, loc3, loc1};

    DcMotor intake;
    DcMotorEx shooter;

    double indexServoPos = .02;
    double flickerServoPos = 0.45;

    @Override
    public void runOpMode() {
        indexServo = hardwareMap.get(Servo.class, "index");
        flickerServo = hardwareMap.get(Servo.class, "flicker");
        loc4 = hardwareMap.get(ColorSensor.class, "location4");
        loc1 = hardwareMap.get(ColorSensor.class, "location1");
        loc2 = hardwareMap.get(ColorSensor.class, "location2");
        loc3 = hardwareMap.get(ColorSensor.class, "location3");

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
            telemetry.addData("sensor 4", "\nred: " + loc4.red() + "\ngreen: " + loc4.green() + "\nblue: " + loc4.blue() + "\nb/r: " + (double)loc4.blue()/ (double)loc4.red() + "\n");
            telemetry.addData("sensor 2", "\nred: " + loc2.red() + "\ngreen: " + loc2.green() + "\nblue: " + loc2.blue() + "\nb/r: " + (double)loc2.blue() / (double)loc2.red() + "\n");
            telemetry.addData("sensor 1", "\nred: " + loc1.red() + "\ngreen: " + loc1.green() + "\nblue: " + loc1.blue() + "\nb/r: " + (double)loc1.blue() / (double)loc1.red() + "\n");
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
        output += "\nSensor 3: \nh:" + hsvValues[0] + "\ns:" + hsvValues[1] + "\nv:" + hsvValues[2] + "\nAlpha: " + loc3.alpha() + ".\n";

        Color.RGBToHSV(loc1.red() * 8, loc1.green() * 8, loc1.blue() * 8, hsvValues);
        output += "\nSensor 1: \nh:" + hsvValues[0] + "\ns:" + hsvValues[1] + "\nv:" + hsvValues[2] + "\nAlpha: " + loc1.alpha() + ".\n";

        return output;
    }


    //..431, .465, .506, .543, .576, .613

}
