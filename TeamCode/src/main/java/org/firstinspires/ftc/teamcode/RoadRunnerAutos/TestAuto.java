package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "Test Auto", group = "Autonomous")
public class TestAuto extends LinearOpMode {
    private MecanumDrive drive;
    private Turntable turntable;
    private Shooter shooter;
    private Camera camera;
    private Intake intake;
    private boolean isFinished = false;

    private Vector2d autoLockingTarget = new Vector2d(-72, 76 * PoseStorage.isRed);
    boolean autoLocking = false;
    private int targetAprilTag = 20;

    public boolean useCamera = false;

    @Override
    public void runOpMode() {
        PoseStorage.isRed = -1;

        if (PoseStorage.isRed == 1) {
            targetAprilTag = 24;
        } else {
            targetAprilTag = 20;
        }

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        turntable = new Turntable(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        camera = new Camera(hardwareMap);

        waitForStart();

        autoLocking = true;
        Actions.runBlocking(new ParallelAction(
                autoLock(),
                new SequentialAction(
                        new SleepAction(12),
                        stopAutoLocking()
                )));

        drive.localizer.update();
        PoseStorage.currentPose = drive.localizer.getPose();

    }

    private Action updatePose() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                drive.localizer.update();
                PoseStorage.currentPose = drive.localizer.getPose();

                if (isFinished) {
                    isFinished = false;
                    return false;
                }

                return true;
            }
        };
    }

    public Action stopUpdate() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                isFinished = true;
                return false;
            }
        };
    }

    private double getAprilTagTurnPower() {
        List<AprilTagDetection> currentDetections = camera.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == targetAprilTag) {
                double tolerance = 0.03;
                double deviation = detection.ftcPose.x;
                if (Math.abs(deviation) > tolerance) {
                    double kP = 0.02;
                    double turnPower = kP * deviation;


                    return Math.max(-0.4, Math.min(0.4, turnPower));
                } else {
                    // We are aligned, so command no turn.
                    return 0.0;
                }

            }
        }
    return 0.0;
    }

    private Action autoLock() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double rotate = getAprilTagTurnPower();

                drive(0, 0, rotate);
                return autoLocking;

            }
        };
    }

    private Action stopAutoLocking() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                autoLocking = false;
                return false;
            }
        };
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
