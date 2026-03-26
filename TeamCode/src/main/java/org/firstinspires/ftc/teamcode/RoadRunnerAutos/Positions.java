package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class Positions {
    // Utility class for all poses in auto and teleop. Uses red side as default.
    public static final Pose2d resetPose = new Pose2d(61.0605, -63.2748, Math.toRadians(90));
    public static final NewVector closeStartPose = new NewVector(-51.7459, 48.5195); //131.9529
    public static final NewVector farStartPose = new NewVector(63.17,14.16); //180
    public static final NewVector intakeClosePoseS = new NewVector(-11.5661, 28.9885); //90
    public static final NewVector intakeMedPoseS = new NewVector(11.9563, 28.9885); //90
    public static final NewVector intakeFarPoseS = new NewVector(35.6717, 28.9885); //90
    public static final NewVector shootPose = new NewVector(-22.8829, 15.6723);// 130.662 for shoot, 180 for scan//132.9229
    public static final NewVector farLeavePose = new NewVector(44.173,15.93); // 153.64
    public static final NewVector farShootPose = new NewVector(50.9675,15.0052); // 153.5

    public static final NewVector leaveShootPose = new NewVector(-26.9057, 12.6696); //-127.4056
    public static final NewVector parkPose = new NewVector(32.8103, -32.8103);

    public static Pose2d getResetPose() {return new Pose2d(resetPose.position.x, resetPose.position.y * PoseStorage.isRed, Math.toRadians(90) * PoseStorage.isRed);}

}
