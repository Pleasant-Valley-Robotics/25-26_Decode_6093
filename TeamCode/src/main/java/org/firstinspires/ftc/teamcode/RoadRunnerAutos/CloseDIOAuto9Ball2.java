package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Config
@Autonomous(name = "Close Dio Auto 9 Ball2", group = "Autonomous")
public class CloseDIOAuto9Ball2 extends LinearOpMode {
    public double timeBeforeStart = 0.0;
    private MecanumDrive drive = null;

    private Vector2d autoLockingTarget = new Vector2d(-72, 76 * PoseStorage.isRed);
    boolean autoLocking = false;
    private Camera camera;
    private int targetAprilTag = 20;

    public boolean useCamera = false;

    @Override
    public void runOpMode() {
        while (!isStopRequested() && !opModeIsActive()) {

            if (gamepad1.dpadUpWasPressed()) {
                timeBeforeStart += 1.0;
            }
            if (gamepad1.dpadDownWasPressed()) {
                timeBeforeStart -= 1.0;
            }

            if (gamepad1.bWasPressed()) {
                PoseStorage.isRed = 1;
            }

            if (gamepad1.xWasPressed()) {
                PoseStorage.isRed = -1;
            }

            if (PoseStorage.isRed == 1) {
                telemetry.addLine("Red");
            } else {
                telemetry.addLine("Blue");
            }
            telemetry.addData("Wait Time", timeBeforeStart);
            telemetry.update();
        }

        double shootAngle = 135*PoseStorage.isRed;
        double leaveShootAngle = 127.4*PoseStorage.isRed;
        double intakeAngle = 90*PoseStorage.isRed;


        drive = new MecanumDrive(hardwareMap, new Pose2d(Positions.getCloseStartPose(), Math.toRadians(131.9529 * PoseStorage.isRed)));
        Turntable turntable = new Turntable(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        camera = new Camera(hardwareMap);

        turntable.addBall(0, Turntable.IndexColors.PURPLE);
        turntable.addBall(1, Turntable.IndexColors.GREEN);
        turntable.addBall(2, Turntable.IndexColors.PURPLE);




        waitForStart();

        if (isStopRequested()) return;

        turntable.updatePosition();

        Actions.runBlocking(new SleepAction(timeBeforeStart));

        //Throttle to speed for launching
        shooter.spinUp(1345);
        // Read april tag
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose()).strafeToLinearHeading(Positions.getShootPose(), Math.toRadians(-180 * PoseStorage.isRed)).build());
        PoseStorage.shotsToCycle = camera.findShotsToCycle();

        if (PoseStorage.isRed == 1) {
            targetAprilTag = 24;
        } else {
            targetAprilTag = 20;
        }

        // Turn back to shoot
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(Positions.getShootPose(), Math.toRadians(shootAngle)).build());


        // Shoot in pattern
        while (!shooter.isAtSpeed()) {
            Actions.runBlocking(new SleepAction(0.1));
        }

        // Turn back on intake to keep in balls while shooting
        intake.setPower(1);
        Actions.runBlocking(new SleepAction(0.05));
        Actions.runBlocking(shooter.shootInPattern(turntable));
        Actions.runBlocking(new SleepAction(0.1));
        shooter.stop();

        // Intake close
        Actions.runBlocking(new ParallelAction(
                intake.autoIntake(camera, turntable),
                new SequentialAction(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(Positions.getIntakeClosePoseS(), Math.toRadians(intakeAngle))
                            .strafeToLinearHeading(Positions.getIntakeClosePoseE(), Math.toRadians(intakeAngle), new TranslationalVelConstraint(4.4)).build()
                )
        ));

        shooter.spinUp(1345);

        // Reverse intake to spit out extra balls
        intake.setPower(-1);
        Actions.runBlocking(new SleepAction(0.05));

        // Drive back to shoot
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(Positions.getShootPose(), Math.toRadians(shootAngle)).build());

        while (!shooter.isAtSpeed()) {
            Actions.runBlocking(new SleepAction(0.1));
        }

        // Turn back on intake to keep in balls while shooting
        intake.setPower(1);
        Actions.runBlocking(new SleepAction(0.05));
        Actions.runBlocking(shooter.shootInPattern(turntable));
        Actions.runBlocking(new SleepAction(0.1));
        shooter.stop();

        // Intake middle
        Actions.runBlocking(new ParallelAction(
                intake.autoIntake(camera, turntable),
                new SequentialAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                .strafeToLinearHeading(Positions.getIntakeMedPoseS(), Math.toRadians(intakeAngle))
                                .strafeToLinearHeading(Positions.getIntakeMedPoseE(), Math.toRadians(intakeAngle), new TranslationalVelConstraint(4.4)).build()
                )
        ));


        shooter.spinUp(1345);

        // Reverse intake to spit out extra balls
        intake.setPower(-1);
        Actions.runBlocking(new SleepAction(0.05));

        // Drive back to shoot
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose()).strafeToLinearHeading(Positions.getLeaveShootPose(), Math.toRadians(leaveShootAngle)).build());


        while (!shooter.isAtSpeed()) {
            Actions.runBlocking(new SleepAction(0.1));
        }

        // Turn back on intake to keep in balls while shooting
        intake.setPower(1);
        Actions.runBlocking(new SleepAction(0.05));
        Actions.runBlocking(shooter.shootInPattern(turntable));
        Actions.runBlocking(new SleepAction(0.1));

        // Stop motors and save to file
        intake.stopIntake();
        shooter.stop();

        drive.updatePoseEstimate();
        PoseStorage.currentPose = drive.localizer.getPose();
        Actions.runBlocking(new SleepAction(0.5));


    }

    private double getAprilTagTurnPower() {
        List<AprilTagDetection> currentDetections = camera.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == targetAprilTag) {
                double tolerance = 0.75; // Tolerance in inches
                double deviation = -detection.ftcPose.z;


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
                return !autoLocking;

            }
        };
    }

    private Action stopAutoLocking() {
        return new Action() {
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
