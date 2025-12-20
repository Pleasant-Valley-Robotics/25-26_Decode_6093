package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;
@TeleOp(name = "Odometry Teleop", group = "StarterBot")
public class OdometryTeleop extends OpMode {
    final double LAUNCHER_FAR_VELOCITY = 1610;
    final double LAUNCHER_CLOSE_VELOCITY = 1400;
    final double LAUNCHER_CYCLE_VELOCITY = 480;

    double CURRENT_TARGET_VELOCITY = 1575;
    boolean manualRotate = true;
    boolean slowMode = false;


    Vector2d parkingPos = null;
    Vector2d closePos = null;
    Vector2d farPos = null;
    Vector2d humanPlayaPos = null;

    int targetAprilTag = 0;


    private FtcDashboard dash = FtcDashboard.getInstance();

    List<Action> driveActions = new ArrayList<>();
    List<Action> shooterActions = new ArrayList<>();

    MecanumDrive drive = null;
    Shooter shooter = null;


    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        shooter = new Shooter(hardwareMap);

        parkingPos = new Vector2d(39.95, -34.17 * PoseStorage.isRed);
        farPos = new Vector2d(49.05, 11.71 * PoseStorage.isRed);
        closePos = new Vector2d(-14.98, 15.181 * PoseStorage.isRed);
        humanPlayaPos = new Vector2d(59, -55.88 * PoseStorage.isRed);

    }

    @Override
    public void loop() {
        drive.updatePoseEstimate();
        TelemetryPacket packet = new TelemetryPacket();

        double rotate = 0;


        if (gamepad2.xWasPressed()) shooterActions.add(shooter.spinUp(LAUNCHER_CLOSE_VELOCITY));
        if (gamepad2.bWasPressed()) shooterActions.add(shooter.spinUp(LAUNCHER_CYCLE_VELOCITY));
        if (gamepad2.yWasPressed()) shooterActions.add(shooter.spinUp(getShotPower()));
        if (gamepad2.aWasPressed()) shooterActions.add(shooter.stopSpin());
        if (gamepad2.right_bumper) shooterActions.add(shooter.fireBall());

        slowMode = gamepad1.left_bumper;
        if (gamepad1.xWasPressed()) manualRotate = !manualRotate;
        if (gamepad1.bWasPressed()) drive.localizer.setPose(new Pose2d(0, 0, Math.toRadians(90 * PoseStorage.isRed)));
        if (gamepad1.aWasPressed()) drive.localizer.setPose(new Pose2d(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, Math.toRadians(90)));

        if (gamepad1.yWasPressed()) {
            TrajectoryActionBuilder goPark = drive.actionBuilder(drive.localizer.getPose()).strafeToLinearHeading(parkingPos, Math.toRadians(180) * PoseStorage.isRed);
            goPark.turn(Math.toRadians(180));
            driveActions.clear();
            driveActions.add(goPark.build());
        }

        if (gamepad1.dpadUpWasPressed()) {
            TrajectoryActionBuilder goClose = drive.actionBuilder(drive.localizer.getPose()).strafeToLinearHeading(closePos, Math.toRadians(142.6656 * PoseStorage.isRed));
            goClose.turn(Math.toRadians(180));
            driveActions.clear();
            driveActions.add(goClose.build());
        }

        if (gamepad1.dpadRightWasPressed()) {
            TrajectoryActionBuilder goFar = drive.actionBuilder(drive.localizer.getPose()).strafeToLinearHeading(farPos, Math.toRadians(152.3424 * PoseStorage.isRed));
            goFar.turn(Math.toRadians(180));
            driveActions.clear();
            driveActions.add(goFar.build());
        }

        if (gamepad1.dpadLeftWasPressed()) {
            TrajectoryActionBuilder goHumanPlaya = drive.actionBuilder(drive.localizer.getPose()).strafeToLinearHeading(humanPlayaPos, Math.toRadians(-90 * PoseStorage.isRed));
            goHumanPlaya.turn(Math.toRadians(180));
            driveActions.clear();
            driveActions.add(goHumanPlaya.build());
        }


        // update running actions
        List<Action> newDriveActions = new ArrayList<>();
        for (Action action : driveActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newDriveActions.add(action);
            }
        }
        driveActions = newDriveActions;

        List<Action> newShooterActions = new ArrayList<>();
        for (Action action : shooterActions) {
            if (action.run(packet)) {
                newShooterActions.add(action);
            }
        }
        shooterActions = newShooterActions;


        if (!driveActions.isEmpty()) {
            rotate = 0;
        } else if (!manualRotate) {
            rotate = autoLockAngle();
        } else {
            rotate = gamepad1.right_stick_x;
        }

        if (driveActions.isEmpty()) {
            driveFieldRelative(gamepad1.left_stick_y, -gamepad1.left_stick_x, rotate);
        }

        telemetry.addData("X position", drive.localizer.getPose().position.x);
        telemetry.addData("Y position", drive.localizer.getPose().position.y);
        telemetry.addData("Heading", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
        telemetry.addData("launch power", getShotPower());
        telemetry.addData("Current launcher speed", shooter.getVelocity());
        telemetry.update();

        PoseStorage.currentPose = drive.localizer.getPose();

        dash.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        shooter.stopSpin();
        drive.leftFront.setPower(0);
        drive.rightFront.setPower(0);
        drive.leftBack.setPower(0);
        drive.rightBack.setPower(0);
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

    public double autoLockAngle() {
        double xDif = -72 - drive.localizer.getPose().position.x;
        double yDif = 72 * PoseStorage.isRed - drive.localizer.getPose().position.y;
        double tolerance = 0.03; // Tolerance in radians

        double rawTargetHeading = Math.atan2(yDif, xDif);
        rawTargetHeading = AngleUnit.normalizeRadians(rawTargetHeading);

        double realTargetHeading = getRealTargetHeading(rawTargetHeading);

        double deviation = drive.localizer.getPose().heading.toDouble() - realTargetHeading;
        deviation = AngleUnit.normalizeRadians(deviation);

        telemetry.addData("Raw Target Heading", Math.toDegrees(rawTargetHeading));
        telemetry.addData("Real Target Heading", Math.toDegrees(realTargetHeading));
        telemetry.addData("X difference", xDif);
        telemetry.addData("Y difference", yDif);
        telemetry.addData("Deviation", Math.toDegrees(deviation));


        if (Math.abs(deviation) > tolerance) {
            double kP = 1.0;
            double turnPower = kP * deviation;


            return Math.max(-0.6, Math.min(0.6, turnPower));
        } else {
            // We are aligned, so command no turn.
            return 0.0;
        }
    }

    public double getRealTargetHeading(double rawHeading) { // Uses velocity to determine a new target heading
        PoseVelocity2d vel = drive.localizer.update();
        return rawHeading;
    }


    public double getShotPower() {
        double xDif = (-72) - drive.localizer.getPose().position.x;
        double yDif =  (72 * PoseStorage.isRed) - drive.localizer.getPose().position.y;
        double distanceFromGoal = Math.sqrt(Math.pow(xDif, 2) + Math.pow(yDif, 2));
        return distanceFromGoal * 12.25;
    }



    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;


        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        if (slowMode){
            maxSpeed = 0.25;
        }

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