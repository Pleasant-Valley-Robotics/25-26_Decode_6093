package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

@Config
@Autonomous(name = "Far Dio Auto 3 Ball (<-- NOT THIS ONE)", group = "Autonomous")
public class FarDIOAuto3Ball extends LinearOpMode {
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

        double shootAngle = 155.83*PoseStorage.isRed;
        int launchVelocity = 1495;


        drive = new MecanumDrive(hardwareMap, new Pose2d(Positions.getFarStartPose(), Math.toRadians(180)));
        intake = new Intake(hardwareMap);
        turntable = new Turntable(hardwareMap);
        shooter = new Shooter(hardwareMap);
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

        // Read motif
        PoseStorage.shotsToCycle = camera.findShotsToCycle();

        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(Positions.getFarShootPose(), Math.toRadians(shootAngle)).build());

        shootBalls(launchVelocity);

        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .strafeToConstantHeading(Positions.getFarLeavePose()).build());

        shooter.stop();

        Actions.runBlocking(new SleepAction(2));

        drive.updatePoseEstimate();
        PoseStorage.currentPose = drive.localizer.getPose();
        //PoseStorage.shotsToCycle = shotsToCycle;


        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(1)
                ));
    }

    private void intakeBalls() {
        intake.setPower(1);
        drive.rightBack.setPower(0.13);
        drive.rightFront.setPower(0.13);
        drive.leftBack.setPower(0.13);
        drive.leftFront.setPower(0.13);

        Actions.runBlocking(
                new SequentialAction(
                    intake.autoIntake(camera, turntable, 5),
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
                shooter.shootInPattern(turntable),
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
