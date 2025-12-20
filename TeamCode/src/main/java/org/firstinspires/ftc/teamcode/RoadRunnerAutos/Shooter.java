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

    final double FEED_TIME = .35;
    final double BACK_TIME = .5;

    private int shotsToCycle = 2;


    private DcMotorEx launcher;
    private CRServo rightFeeder;
    private CRServo leftFeeder;
    ElapsedTime feederTimer = new ElapsedTime();


    private AprilTagProcessor aprilTag;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;






    public Shooter(HardwareMap hardwareMap) {
        initAprilTag(hardwareMap);

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setDirection(DcMotor.Direction.REVERSE);

        launcher.setZeroPowerBehavior(BRAKE);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        leftFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "leftFeeder");

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFeeder.setDirection(DcMotorSimple.Direction.FORWARD);


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

    public int findShotsToCycle() {
        int shotsToCycle = 0;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (!currentDetections.isEmpty()) {
            for (int i = 0; i < currentDetections.size(); i++) {
                if (currentDetections.get(i).id == 22) {
                    shotsToCycle = 2;
                    break;
                } else if (currentDetections.get(i).id == 23) {
                    shotsToCycle = 1;
                }
            }
        }
        return shotsToCycle;
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
            private boolean movingForward = false;

            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                if (!movingForward) {
                    movingForward = true;

                    rightFeeder.setPower(1);
                    leftFeeder.setPower(1);

                    feederTimer.reset();

                    return true;
                }

                if (feederTimer.seconds() > BACK_TIME + FEED_TIME) {
                    rightFeeder.setPower(0);
                    leftFeeder.setPower(0);

                    return false;
                } else if (feederTimer.seconds() > BACK_TIME) {
                    rightFeeder.setPower(-1);
                    leftFeeder.setPower(-1);
                    return true;
                }

                return true;

            }

        };
    }


    private void initAprilTag(HardwareMap hardwareMap) {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }


        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }

    public double getVelocity() {return launcher.getVelocity();}

}
