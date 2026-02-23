package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.CancelableProfile;
import com.acmerobotics.roadrunner.CompositeVelConstraint;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "Test Auto", group = "Autonomous")
public class TestAuto extends LinearOpMode {
    MecanumDrive drive;
    Camera camera;

    int targetAprilTag = 24;
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(Positions.getCloseStartPose(), Math.toRadians(131.9529 * PoseStorage.isRed)));
        camera = new Camera(hardwareMap);
        Actions.runBlocking(driveAutoLocking());
    }


    private Action driveAutoLocking() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                drive(0, 0, autoLockPower());
                return true;
            }
        };
    }

    private double autoLockPower() {
        double tolerance = 0.05; // Tolerance in radians;
        double deviation = 0;

        List<LLResultTypes.FiducialResult> currentDetections = camera.getDetections();
        // Use camera for final auto-locking
        for (LLResultTypes.FiducialResult detection : currentDetections) {
            if (detection != null && detection.getFiducialId() == targetAprilTag) {
                deviation = detection.getTargetXDegrees();
            }
        }

        if (Math.abs(deviation) > tolerance) {
            double kP = 0.02;
            double turnPower = kP * deviation;


            return Math.max(-0.4, Math.min(0.4, turnPower));
        } else {
            // We are aligned, so command no turn.
            return 0.0;
        }
    }

    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;


        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        drive.leftFront.setPower(maxSpeed * (frontLeftPower / maxPower));
        drive.rightFront.setPower(maxSpeed * (frontRightPower / maxPower));
        drive.leftBack.setPower(maxSpeed * (backLeftPower / maxPower));
        drive.rightBack.setPower(maxSpeed * (backRightPower / maxPower));
    }

}
