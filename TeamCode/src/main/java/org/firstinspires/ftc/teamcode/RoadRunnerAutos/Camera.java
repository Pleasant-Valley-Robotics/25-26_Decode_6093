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

    public ColorSensor loc4;
    public ColorSensor loc3;
    public ColorSensor loc2;
    public ColorSensor loc1;
    private final int GREEN_HUE = 140;
    private final int PURPLE_HUE = 150;

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

        loc4 = hardwareMap.get(ColorSensor.class, "location4");
        loc3 = hardwareMap.get(ColorSensor.class, "location3");
        loc2 = hardwareMap.get(ColorSensor.class, "location2");
        loc1 = hardwareMap.get(ColorSensor.class, "location1");



    }


    public int findShotsToCycle() {
        // Amount of times to turn the turntable LEFT
        if (!aprilTag.getDetections().isEmpty()) {
            switch (aprilTag.getDetections().get(0).id) {
                case 21:
                    return 0;
                case 22:
                    return 2;
                case 23:
                    return 1;
            }
        }
        return 0;

    }

    public Turntable.IndexColors getBallColor() {
        float[] hsvValues = {0,0,0};

        Color.RGBToHSV(loc4.red() * 8, loc4.green() * 8, loc4.blue() * 8, hsvValues);

        if (Math.abs(hsvValues[0] - PURPLE_HUE) < 5) {
            return Turntable.IndexColors.PURPLE;
        } else if (Math.abs(hsvValues[0] - GREEN_HUE) < 5) {
            return Turntable.IndexColors.GREEN;
        }

        return Turntable.IndexColors.PURPLE;
    }

    public boolean ballDetected() {
        return loc4.alpha() > 40;
    }

//    public Turntable.IndexColors[] scan() {
//        float[] hsv1 = {0,0,0};
//        float[] hsv2 = {0,0,0};
//        float[] hsv3 = {0,0,0};
//
//        Color.RGBToHSV(loc1.red() * 8, loc1.green() * 8, loc1.blue() * 8, hsv1);
//        Color.RGBToHSV(loc2.red() * 8, loc2.green() * 8, loc2.blue() * 8, hsv2);
//        Color.RGBToHSV(loc3.red() * 8, loc3.green() * 8, loc3.blue() * 8, hsv3);
//
//        if (hsv1[0] > hsv2[0] && hsv1[0] > hsv3[0]) {
//            Turntable.IndexColors.PURPLE
//        } else if (hsv2[0] > hsv3[0]) {
//
//        } else {
//
//        }
//
//    }




}
