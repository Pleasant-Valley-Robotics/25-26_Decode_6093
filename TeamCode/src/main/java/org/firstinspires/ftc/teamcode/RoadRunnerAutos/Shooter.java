package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.features2d.Feature2D;

import java.lang.Math;
import java.util.List;

public class Shooter {

    private DcMotorEx launcher;
    ElapsedTime feederTimer = new ElapsedTime();

    public Shooter(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, "shooter");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setDirection(DcMotor.Direction.REVERSE);

        launcher.setZeroPowerBehavior(BRAKE);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));


    }
    public Action spinUp(double targetSpeed) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    launcher.setVelocity(targetSpeed);
                    initialized = true;
                }

                double vel = launcher.getVelocity();
                packet.put("Shooter Velocity", vel);
                return 10 > Math.abs(vel - targetSpeed);

            }
        };

    }


    public Action stopSpin() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    launcher.setVelocity(0.0);
                    initialized = true;
                }

                double vel = launcher.getVelocity();
                packet.put("Shooter Velocity", vel);
                return vel != 0.0;

            }
        };

    }

    public Action fireBall() {
        return new Action() {


            @Override
            public boolean run (@NonNull TelemetryPacket packet) {


                return true;

            }

        };
    }

    public double getVelocity() {return launcher.getVelocity();}

}
