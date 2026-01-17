package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.DIO;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class Camera {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private ColorSensor colorSensor;
    private final int GREEN_HUE = 160;
    private final int PURPLE_HUE = 210;

    public Camera(HardwareMap hardwareMap) {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

        colorSensor = hardwareMap.get(ColorSensor.class, "location4");


    }


    public int findShotsToCycle() {
        // Amount of times to turn the turntable LEFT
        return aprilTag.getDetections().get(0).id - 21;
    }

    public Turntable.IndexColors getBallColor() {
        float[] hsvValues = {0,0,0};

        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        if (Math.abs(hsvValues[0] - PURPLE_HUE) < 25) {
            return Turntable.IndexColors.PURPLE;
        } else if (Math.abs(hsvValues[0] - GREEN_HUE) < 25) {
            return Turntable.IndexColors.GREEN;
        }

        return null;
    }

    public boolean ballDetected() {
        return colorSensor.alpha() > 32;
    }




}
