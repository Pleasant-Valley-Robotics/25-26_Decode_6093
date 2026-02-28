package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RoadRunnerAutos.Camera;
import org.firstinspires.ftc.teamcode.RoadRunnerAutos.Intake;
import org.firstinspires.ftc.teamcode.RoadRunnerAutos.PoseStorage;
import org.firstinspires.ftc.teamcode.RoadRunnerAutos.Positions;
import org.firstinspires.ftc.teamcode.RoadRunnerAutos.Shooter;
import org.firstinspires.ftc.teamcode.RoadRunnerAutos.Turntable;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Disabled
@TeleOp(name="Velocity Based Autolocking", group="Odometry")
@Config
public class VelocityBased extends OpMode {
    private MecanumDrive drive;
    private Intake intake;
    private Turntable turntable;
    private Camera camera;
    private Shooter shooter;

    private final Vector2d autoLockingTarget = new Vector2d(-72, 76 * PoseStorage.isRed);
    private int targetAprilTag;
    private boolean manualRotate;

    public static double velocityCoefficientX = 1;
    public static double velocityCoefficientY = 1;
    public static int flyWheelSpeed = 1200;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose); // Pass saved pose from autos
        turntable = new Turntable(hardwareMap);
        intake = new Intake(hardwareMap);
        camera = new Camera(hardwareMap);
        shooter = new Shooter(hardwareMap);

        if (PoseStorage.isRed == 1) {
            targetAprilTag = 24;
        } else {
            targetAprilTag = 20;
        }
        manualRotate = true;
    }

    @Override
    public void start() {
        turntable.updatePosition();
    }

    @Override
    public void loop() {
        PoseVelocity2d velocity = drive.updatePoseEstimate();
        Pose2d position = drive.localizer.getPose();
        Vector2d target = updateTarget(velocity);
        double distanceFromGoal = findDistance();


        if (gamepad2.xWasPressed()) {turntable.turnToPosition(2);}
        if (gamepad2.yWasPressed()) {turntable.turnToPosition(1);}
        if (gamepad2.bWasPressed()) {turntable.turnToPosition(0);}
        if (gamepad2.right_bumper) {turntable.extraRange(false);}
        else if (gamepad2.rightBumperWasReleased()) {turntable.extraRange(true);}
        if (gamepad2.right_trigger > 0) shooter.spinUp(flyWheelSpeed);
        if (gamepad2.left_trigger > 0) shooter.stop();
        if (gamepad2.dpadDownWasPressed()) {shooter.setServoPos(shooter.downPos);}
        if (gamepad2.dpadUpWasPressed()) {shooter.setServoPos(shooter.upPos);}
        intake.setPower(gamepad2.left_stick_y);

        if (gamepad1.aWasPressed()) {camera.setEnabled(!camera.isEnabled());}
        if (gamepad1.xWasPressed()) {manualRotate = !manualRotate;}
        if (gamepad1.dpadLeftWasPressed()) drive.localizer.setPose(Positions.getResetPose());

        double rotate;
        if (!manualRotate) {
            rotate = autoLock(target);
        } else {
            rotate = gamepad1.right_stick_x;
        }

        List<LLResultTypes.FiducialResult> aprilTags = camera.getDetections();
        String aprilTagS = "";
        for (LLResultTypes.FiducialResult aprilTag : aprilTags) {
            aprilTagS += aprilTag.getFiducialId() + ", ";
        }

        driveFieldRelative(gamepad1.left_stick_y, -gamepad1.left_stick_x, rotate);
        PoseStorage.currentPose = drive.localizer.getPose();

        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("Distance from goal", distanceFromGoal);
        telemetry.addData("Flywheel target speed", flyWheelSpeed);
        telemetry.addData("Apriltags IDs", aprilTagS);
        telemetry.addLine();
        telemetry.addData("Using camera?", camera.isEnabled());
        telemetry.addData("Manual rotate?", manualRotate);
        telemetry.addLine();
        telemetry.addData("Position according to limelight", "\n\tX Pose: %f" +
                "\n\tY Pose: %f", camera.getLLPose().position.x, camera.getLLPose().position.y);
        if (!aprilTags.isEmpty()) {
            telemetry.addData("Position relative to aprilTag[0]", "\n\tX Pose: %f" +
                    "\n\tY Pose: %f", aprilTags.get(0).getCameraPoseTargetSpace().getPosition().x, aprilTags.get(0).getCameraPoseTargetSpace().getPosition().y);
        }
        telemetry.addData("Target (adjusted for velocity)", "\n\tX Target: %f" +
                "\n\tY Target: %f", target.x, target.y);
        telemetry.addLine();
        telemetry.addData("Current velocity", "\n\tX Velocity: %f" +
                "\n\tY Velocity: %f" +
                "\n\tHeading Velocity: %f", velocity.linearVel.x, velocity.linearVel.y, velocity.angVel);
        telemetry.addData("Current position", "\n\tX Position: %f" +
                "\n\tY Position: %f" +
                "Heading: %f", position.position.x, position.position.y, Math.toDegrees(position.heading.toDouble()));
        telemetry.addData("Using camera?", camera.isEnabled());
    }

    @Override
    public void stop() {
        // Stop all servos and set motor power to brake
        drive.leftFront.setPower(0);
        drive.rightFront.setPower(0);
        drive.leftBack.setPower(0);
        drive.rightBack.setPower(0);
    }

    public Vector2d updateTarget(PoseVelocity2d velocity) {
        double newX = autoLockingTarget.x + (velocity.linearVel.x * velocityCoefficientX);
        double newY = autoLockingTarget.y + (velocity.linearVel.y * velocityCoefficientY);

        return new Vector2d(newX, newY);
    }


    public double autoLock(Vector2d target) {
        double tolerance = 0.1;
        double kP = 0.1;

        if (camera.isEnabled()) {
            List<LLResultTypes.FiducialResult> currentDetections = camera.getDetections();
            for (LLResultTypes.FiducialResult detection : currentDetections) {
                if (detection != null && detection.getFiducialId() == targetAprilTag) {
                    double error = detection.getTargetXDegrees();
                    if (Math.abs(error) > tolerance) {
                        double turnPower = kP * error;

                        return Math.max(-0.6, Math.min(0.6, turnPower));
                    } else {
                        return 0.0;
                    }
                }
            }
        }

        double xDif = target.x - drive.localizer.getPose().position.x;
        double yDif = target.y - drive.localizer.getPose().position.y;

        double rawTargetHeading = Math.atan2(yDif, xDif);
        rawTargetHeading = AngleUnit.normalizeRadians(rawTargetHeading);

        double realTargetHeading = AngleUnit.normalizeRadians(rawTargetHeading);
        double error = drive.localizer.getPose().heading.toDouble() - realTargetHeading;

        error = AngleUnit.normalizeRadians(error);

        if (Math.abs(error) > tolerance) {
            double turnPower = kP * error;

            return Math.max(-0.6, Math.min(0.6, turnPower));
        } else {
            return 0.0;
        }

    }

    private double findDistance() {
        double xDif = (-72) - drive.localizer.getPose().position.x;
        double yDif =  (72 * PoseStorage.isRed) - drive.localizer.getPose().position.y;
        return Math.sqrt(Math.pow(xDif, 2) + Math.pow(yDif, 2));
    }


    // Copied from FieldCentric.java
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                drive.localizer.getPose().heading.toDouble() - Math.toRadians(90 * PoseStorage.isRed));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
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
