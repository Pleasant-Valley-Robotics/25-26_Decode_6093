package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

@Config
@Autonomous(name = "Close 9", group = "Close")
public class CloseDIOAuto9Ball extends LinearOpMode {
    public double timeBeforeStart = 0.0;
    private MecanumDrive drive;
    Turntable turntable;
    Shooter shooter;
    Intake intake;
    Camera camera;
    int targetAprilTag = 24;


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
                targetAprilTag = 24;
            }

            if (gamepad1.xWasPressed()) {
                PoseStorage.isRed = -1;
                targetAprilTag = 20;
            }

            if (PoseStorage.isRed == 1) {
                telemetry.addLine("Red");
            } else {
                telemetry.addLine("Blue");
            }
            telemetry.addData("Wait Time", timeBeforeStart);
            telemetry.update();
        }

        double shootAngle = 134.5*PoseStorage.isRed;
        double leaveShootAngle = 127.4*PoseStorage.isRed;
        double intakeAngle = 90*PoseStorage.isRed;
        int launchVelocity = 1239;


        drive = new MecanumDrive(hardwareMap, new Pose2d(Positions.closeStartPose.getVector2d(), Math.toRadians(131.9529 * PoseStorage.isRed)));
        turntable = new Turntable(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        camera = new Camera(hardwareMap);


        turntable.addBall(0, Turntable.IndexColors.PURPLE);
        turntable.addBall(1, Turntable.IndexColors.GREEN);
        turntable.addBall(2, Turntable.IndexColors.PURPLE);

        waitForStart();

        if (isStopRequested()) return;
        shooter.setServoPos(shooter.downPos);
        turntable.updatePosition();

        Actions.runBlocking(new SleepAction(timeBeforeStart));


        shooter.spinUp(launchVelocity);

        //Drive to shoot
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(Positions.farShootPose.getVector2d(), Math.toRadians(-180 * PoseStorage.isRed),null,new ProfileAccelConstraint(-30,70)).build());


        // Read motif
        PoseStorage.shotsToCycle = camera.findShotsToCycle();

        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(Positions.shootPose.getVector2d(), Math.toRadians(shootAngle)).build());

        shootBalls(launchVelocity);

        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose()).strafeToLinearHeading(Positions.intakeClosePoseS.getVector2d(), Math.toRadians(intakeAngle)).build());

        intakeBalls(4.25);

        shooter.spinUp(launchVelocity + 35);

        // Drive to shoot
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose()).strafeToLinearHeading(Positions.leaveShootPose.getVector2d(), Math.toRadians(leaveShootAngle)).build());

        turntable.addBall(0, Turntable.IndexColors.PURPLE);
        turntable.addBall(1, Turntable.IndexColors.GREEN);
        turntable.addBall(2, Turntable.IndexColors.PURPLE);

        shootBalls(launchVelocity + 35);

        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose()).strafeToLinearHeading(Positions.intakeMedPoseS.getVector2d(), Math.toRadians(intakeAngle)).build());
        intakeBalls(4.25);

        shooter.spinUp(launchVelocity + 30);

        // Drive to shoot
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose()).strafeToLinearHeading(Positions.leaveShootPose.getVector2d(), Math.toRadians(leaveShootAngle)).build());

        if (PoseStorage.isRed == 1) {
            turntable.addBall(0, Turntable.IndexColors.GREEN);
            turntable.addBall(1, Turntable.IndexColors.PURPLE);
            turntable.addBall(2, Turntable.IndexColors.PURPLE);
        } else {
            turntable.addBall(0, Turntable.IndexColors.PURPLE);
            turntable.addBall(1, Turntable.IndexColors.PURPLE);
            turntable.addBall(2, Turntable.IndexColors.GREEN);
        }
        shootBalls(launchVelocity + 30);

        shooter.stop();
        intake.stopIntake();

        Actions.runBlocking(new SleepAction(2));

        drive.updatePoseEstimate();
        PoseStorage.currentPose = drive.localizer.getPose();
        //PoseStorage.shotsToCycle = shotsToCycle;


        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(1)
                ));
    }

    private void intakeBalls(double limitTime) {
        intake.setPower(1);
        drive.rightBack.setPower(0.13);
        drive.rightFront.setPower(0.13);
        drive.leftBack.setPower(0.13);
        drive.leftFront.setPower(0.13);

        Actions.runBlocking(
                new SequentialAction(
                    intake.autoIntake(camera, turntable, limitTime),
                    intake.reverse()
                )
        );

        drive.rightBack.setPower(0);
        drive.rightFront.setPower(0);
        drive.leftBack.setPower(0);
        drive.leftFront.setPower(0);

        drive.updatePoseEstimate();
    }

    private void shootBalls(int speed) {

        Actions.runBlocking(new RaceAction(
                getToSpeed(speed),
                new SleepAction(1)
        ));

        intake.setPower(1);
        Actions.runBlocking(new RaceAction(
                shooter.shootInPattern(turntable, 0.22),
                driveAutoLocking()
                )
        );

        turntable.clear();

        intake.stopIntake();
        shooter.stop();
    }


    private Action getToSpeed(int speed) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shooter.spinUp(speed);
                return !shooter.isAtSpeed();
            }
        };
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
        double tolerance = 0; // Tolerance in radians;
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
