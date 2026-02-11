package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class PoseStorage {
    // It took way too long to figure out how to do this
    public static int shotsToCycle = 0; // Assuming GPP as default
    public static Pose2d currentPose = new Pose2d(0,0,0);
    public static int isRed = 1; //1 for red, -1 for blue (so we can multiply for opposite auto's)
}