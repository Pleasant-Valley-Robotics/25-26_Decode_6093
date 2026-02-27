package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class Positions {
    // Utility class for all poses in auto and teleop. Uses red side as default.
    private static final Pose2d resetPose = new Pose2d(61.0605, -63.2748, Math.toRadians(90));
    private static final Vector2d closeStartPose = new Vector2d(-51.7459, 48.5195); //131.9529
    private static final Vector2d farStartPose = new Vector2d(63.17,14.16); //180
    private static final Vector2d intakeClosePoseS = new Vector2d(-11.5661, 28.9885); //90
    private static final Vector2d intakeMedPoseS = new Vector2d(11.9563, 28.9885); //90
    private static final Vector2d intakeFarPoseS = new Vector2d(35.6717, 28.9885); //90
    private static final Vector2d shootPose = new Vector2d(-22.8829, 15.6723);// 130.662 for shoot, 180 for scan//132.9229
    private static final Vector2d farShootPose = new Vector2d(44.173,15.93); // 153.64
    private static final Vector2d leaveShootPose = new Vector2d(-26.9057, 12.6696); //-127.4056
    private static final Vector2d parkPose = new Vector2d(32.8103, -32.8103);

    private static double getDistDif() {
        if (PoseStorage.isRed == -1) {
            return -2.2;
        } else {
            return -2.2;
        }
    }
    public static Pose2d getResetPose() {return new Pose2d(resetPose.position.x, resetPose.position.y * PoseStorage.isRed, Math.toRadians(90) * PoseStorage.isRed);}
    public static Vector2d getResetPoseV() {return new Vector2d(resetPose.position.x, resetPose.position.y * PoseStorage.isRed);}
    public static Vector2d getLeaveShootPose() {return new Vector2d(leaveShootPose.x, leaveShootPose.y * PoseStorage.isRed);}
    public static Vector2d getCloseStartPose() {return new Vector2d(closeStartPose.x, closeStartPose.y * PoseStorage.isRed);}
    public static Vector2d getShootPose() {return new Vector2d(shootPose.x, shootPose.y * PoseStorage.isRed);}
    public static Vector2d getParkPose() {return new Vector2d(parkPose.x, parkPose.y * PoseStorage.isRed);}
    public static Vector2d getFarStartPose() {return new Vector2d(farStartPose.x,farStartPose.y * PoseStorage.isRed);}
    public static Vector2d getFarShootPose() {return new Vector2d(farShootPose.x,farShootPose.y * PoseStorage.isRed);}

    public static Vector2d getIntakeClosePoseS() {return new Vector2d(intakeClosePoseS.x + getDistDif(), intakeClosePoseS.y * PoseStorage.isRed);}
    public static Vector2d getIntakeMedPoseS() {return new Vector2d(intakeMedPoseS.x + getDistDif(), intakeMedPoseS.y * PoseStorage.isRed);}
    public static Vector2d getIntakeFarPoseS() {return new Vector2d(intakeFarPoseS.x + getDistDif(), intakeFarPoseS.y * PoseStorage.isRed);}

}
