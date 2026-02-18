package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.DIO;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class Camera {
    private Limelight3A limelight;
    private ColorSensor loc4;
    private ColorSensor loc1;

    private final int PURPLE_HUE = 180;

    private boolean isEnabled = true;

    public Camera(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        loc4 = hardwareMap.get(ColorSensor.class, "location4");
        loc1 = hardwareMap.get(ColorSensor.class, "location1");

        limelight.pipelineSwitch(0);
        limelight.start();


    }


    public int findShotsToCycle() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
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

        return 0;

    }

    public List<LLResultTypes.FiducialResult> getDetections() {
        return limelight.getLatestResult().getFiducialResults();
    }

    public Turntable.IndexColors getBallColor() {
        float[] hsvValues = {0,0,0};

        Color.RGBToHSV(loc1.red() * 8, loc1.green() * 8, loc1.blue() * 8, hsvValues);

        if (hsvValues[0] > PURPLE_HUE) {
            return Turntable.IndexColors.PURPLE;
        } else {
            return Turntable.IndexColors.GREEN;
        }
    }

    public boolean isEnabled() {
        return isEnabled;
    }

    public void setEnabled(boolean enabled) {
        isEnabled = enabled;
    }

    public boolean ballDetected() {
        return loc4.alpha() >= 36.5;
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
