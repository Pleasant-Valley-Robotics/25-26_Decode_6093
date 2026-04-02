package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import android.graphics.Color;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.DIO;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class Camera {
    private Limelight3A limelight;
    private ColorSensor loc4;
    private DistanceSensor loc1;
    private DistanceSensor loc2;

    private final int PURPLE_HUE = 180;

    private boolean isEnabled = true;

    public Camera(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        loc1 = hardwareMap.get(DistanceSensor.class, "location1");
        loc2 = hardwareMap.get(DistanceSensor.class, "location2");

        limelight.pipelineSwitch(0);
        limelight.start();


    }


    public int findShotsToCycle() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                switch (fiducials.get(0).getFiducialId()) {
                    case 21:
                        return 0;
                    case 22:
                        return 2;
                    case 23:
                        return 1;
                }
            }
        }

        // Amount of times to turn the turntable LEFT

        return -1;

    }

    public Pose2d getLLPose() {
        Pose3D raw = limelight.getLatestResult().getBotpose();

        return new Pose2d(raw.getPosition().y * 39.37, raw.getPosition().x * 39.37, raw.getOrientation().getYaw(AngleUnit.DEGREES));
    }

    public List<LLResultTypes.FiducialResult> getDetections() {
        return limelight.getLatestResult().getFiducialResults();
    }


    public boolean isEnabled() {
        return isEnabled;
    }

    public void setEnabled(boolean enabled) {
        isEnabled = enabled;
    }

    public boolean ballDetectedR() {
        return loc1.getDistance(DistanceUnit.MM) < 135;
    }

    public boolean ballDetectedL() {
        return loc2.getDistance(DistanceUnit.MM) < 190;
    }



}
