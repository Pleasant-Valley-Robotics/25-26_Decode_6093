package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class Positions {
    // Utility class for all poses in auto and teleop. Uses red side as default.
    private static final Pose2d resetPose = new Pose2d(73.9877, -69.7049, Math.toRadians(90));
    private static final Vector2d closeStartPose = new Vector2d(-57.0026, 59.3618); //131.9529
    private static final Vector2d intakeClosePoseS = new Vector2d(-8.3053, 35); //90
    private static final Vector2d intakeClosePoseE = new Vector2d(intakeClosePoseS.x, intakeClosePoseS.y + 20); //90

    private static final Vector2d intakeMedPoseS = new Vector2d(18.9796, 35); //90
    private static final Vector2d intakeMedPoseE = new Vector2d(intakeMedPoseS.x, intakeMedPoseS.y + 20); //90

    private static final Vector2d intakeFarPoseS = new Vector2d(45.6425, 35); //90
    private static final Vector2d intakeFarPoseE = new Vector2d(intakeFarPoseS.x, intakeFarPoseS.y + 20); //90

    private static final Vector2d shootPose = new Vector2d(-20.2168, 19.232);// 130.662 for shoot, 180 for scan
    private static final Vector2d leaveShootPose = new Vector2d(-28.5357, 16.0285);
    private static final Vector2d parkPose = new Vector2d(47.9226, -35.0659);

    public static Pose2d getResetPose() {return new Pose2d(resetPose.position.x, resetPose.position.y * PoseStorage.isRed, Math.toRadians(90) * PoseStorage.isRed);}
    public static Vector2d getResetPoseV() {return new Vector2d(resetPose.position.x, resetPose.position.y * PoseStorage.isRed);}
    public static Vector2d getLeaveShootPose() {return new Vector2d(leaveShootPose.x, leaveShootPose.y * PoseStorage.isRed);}
    public static Vector2d getCloseStartPose() {return new Vector2d(closeStartPose.x, closeStartPose.y * PoseStorage.isRed);}
    public static Vector2d getShootPose() {return new Vector2d(shootPose.x, shootPose.y * PoseStorage.isRed);}
    public static Vector2d getParkPose() {return new Vector2d(parkPose.x, parkPose.y * PoseStorage.isRed);}

    public static Vector2d getIntakeClosePoseS() {return new Vector2d(intakeClosePoseE.x + (-6 * PoseStorage.isRed), intakeClosePoseS.y * PoseStorage.isRed);}
    public static Vector2d getIntakeClosePoseE() {return new Vector2d(intakeClosePoseE.x + (-6 * PoseStorage.isRed), intakeClosePoseE.y * PoseStorage.isRed);}
    public static Vector2d getIntakeMedPoseS() {return new Vector2d(intakeMedPoseE.x + (-6 * PoseStorage.isRed), intakeMedPoseS.y * PoseStorage.isRed);}
    public static Vector2d getIntakeMedPoseE() {return new Vector2d(intakeMedPoseE.x + (-6 * PoseStorage.isRed), intakeMedPoseE.y * PoseStorage.isRed);}
    public static Vector2d getIntakeFarPoseS() {return new Vector2d(intakeFarPoseE.x + (-6 * PoseStorage.isRed), intakeFarPoseS.y * PoseStorage.isRed);}
    public static Vector2d getIntakeFarPoseE() {return new Vector2d(intakeFarPoseE.x + (-6 * PoseStorage.isRed), intakeFarPoseE.y * PoseStorage.isRed);}

}
