package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;
@TeleOp(name="ODODIO", group="Odometry")
@Config
public class ODODIO extends OpMode {

    // Initialize systems to null, these are set in init
    private MecanumDrive drive;
    Turntable turntable;
    Intake intake;
    Camera camera;
    Shooter shooter;


    // Drive modes
    private boolean manualRotate = true;
    private boolean slowMode = false;

    // Initialize all positions to null, these are set in init based on blue or red
    private Vector2d parkingVec;
    private Vector2d closeVec;
    private Vector2d farVec;
    private Vector2d humanPlayaVec;

    private Pose2d parkingPose;


    // List of actions to be performed each tick, updated continually
    // driveActions should always be either 1 or 0 long, systemsActions can have multiple
    private List<Action> driveActions = new ArrayList<>();
    private List<Action> systemsActions = new ArrayList<>();

    private Gamepad prevGamepad1 = new Gamepad();
    private Gamepad prevGamepad2 = new Gamepad();

    private double intakeTimestamp = 0;

    // Dash used for sending telemetry every loop
    private FtcDashboard dash = FtcDashboard.getInstance();

    public static int flyWheelSpeed = 1500;
    private double distanceFromGoal;
    Vector2d autoLockingTarget = new Vector2d(-72, 72 * PoseStorage.isRed);


    @Override
    public void init() {
        // Declare all systems so they can be used in loop
        // This has to be done here because hardwareMap doesn't exist until startup
        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose); // Pass saved pose from autos
        turntable = new Turntable(hardwareMap);
        intake = new Intake(hardwareMap);
        camera = new Camera(hardwareMap);
        shooter = new Shooter(hardwareMap);

        // Set positions based on whether or not we are red
        // isRed is a integer, not boolean. Either -1 or 1
        parkingPose = new Pose2d(42.7961, -39.4571 * PoseStorage.isRed, 0);
        parkingVec = new Vector2d(42.7961, -39.4571 * PoseStorage.isRed);
        farVec = new Vector2d(55.9338, -1.024 * PoseStorage.isRed); // 147.4461
        closeVec = new Vector2d(-23.9816, 14.2421 * PoseStorage.isRed); // 133.104
        humanPlayaVec = new Vector2d(71.2383, -63.4445 * PoseStorage.isRed);

        prevGamepad1.copy(gamepad1);
        prevGamepad2.copy(gamepad2);

        for (int i = 0; i < 30; i++) {
            systemsActions.add(null);
        }
    }

    @Override
    public void loop() {
        drive.updatePoseEstimate(); // Get pose from odometry
        distanceFromGoal = updateDistanceFromGoal(drive);
        // Used to print to console and get field overlay, no clue how tho
        TelemetryPacket packet = new TelemetryPacket();
        flyWheelSpeed = (int) (-0.000849d * Math.pow(distanceFromGoal, 3) + 0.3294 * Math.pow(distanceFromGoal, 2) - 35.4967 * distanceFromGoal + 2474.4525);

        // Gamepad 2 Controls:
        // R-Stick Y: power intake
        // R-Bumper: Clockwise
        // L-Bumper: Rotate counter-clockwise
        if (gamepad2.right_stick_y != 0) {
            if (gamepad2.right_stick_y > 0 && prevGamepad2.right_stick_y == 0) {
                shooter.setServoPos(shooter.downPos);
                systemsActions.set(0, intake.autoIntake(camera, turntable));
            }
            intake.setPower(gamepad2.right_stick_y);
        } else {
            systemsActions.set(0, null);
            intake.stopIntake();
        }

        if (gamepad2.aWasPressed()) {
            shooter.setServoPos(shooter.downPos);
            systemsActions.set(1, shooter.fireOnce(turntable));
        }
        if (gamepad2.xWasPressed()) {
            shooter.setServoPos(shooter.downPos);
            systemsActions.set(2, shooter.shootAll(turntable));
        }

        if (gamepad2.rightBumperWasPressed()) {
            shooter.setServoPos(shooter.downPos);
            turntable.turnLeft();
        }
        if (gamepad2.leftBumperWasPressed()) {
            shooter.setServoPos(shooter.downPos);
            turntable.turnRight();
        }
        if (gamepad2.right_trigger > 0) shooter.spinUp(flyWheelSpeed);
        if (gamepad2.left_trigger > 0) shooter.stop();
        if (gamepad2.dpadUpWasPressed()) shooter.setServoPos(shooter.upPos);
        if (gamepad2.dpadDownWasPressed()) shooter.setServoPos(shooter.downPos);
        if (gamepad2.bWasPressed()) {
            shooter.setServoPos(shooter.downPos);
            systemsActions.set(3, shooter.shootInPattern(turntable));
        }

        if (gamepad2.dpadRightWasPressed()) {
            for (int i = 0; i < 3; i++) {
                double xCor = (Math.random() * 120) - 60;
                double yCor = (Math.random() * 80) - 40;
                double heading = (Math.random() * 360) - 180;

                Action driveRandom = drive.actionBuilder(drive.localizer.getPose()).splineTo(new Vector2d(xCor, yCor), heading).build();

                driveActions.set(0, driveRandom);
            }

        }

        if (shooter.getLastSpeed() != 0) {
            shooter.spinUp(flyWheelSpeed);
        }

        if (shooter.isAtSpeed() && shooter.isMoving()) {
            gamepad2.rumble(1);
        }

        double rotate = 0;

        // Gamepad 1 controls:
        // X: toggle between manual rotation and auto locking
        // B: reset robot position to 0, 0. This will eventually probably be a combo of buttons so it can't be accidentally pressed
        // A: reset robot heading so forward is where we are facing

        if (gamepad1.xWasPressed()) manualRotate = !manualRotate;
        if (gamepad1.dpadLeftWasPressed()) drive.localizer.setPose(new Pose2d(0, 0, Math.toRadians(90 * PoseStorage.isRed)));
        //if (gamepad1.aWasPressed()) drive.localizer.setPose(new Pose2d(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, Math.toRadians(90)));
        if (gamepad1.right_trigger > 0) driveActions.clear();

        if (gamepad2.yWasPressed()) systemsActions.set(4, shooter.shootAllFAST(turntable));
        slowMode = gamepad1.left_trigger > 0;

        if (gamepad1.dpadUpWasPressed()) {
            TrajectoryActionBuilder goClose = drive.actionBuilder(drive.localizer.getPose()).strafeToSplineHeading(closeVec, Math.toRadians(130.5 * PoseStorage.isRed),null,new ProfileAccelConstraint(-20,50));
            driveActions.clear();
            driveActions.add(goClose.build());
        }
        if (gamepad1.dpadRightWasPressed()) {
            TrajectoryActionBuilder goPark = drive.actionBuilder(drive.localizer.getPose()).strafeToSplineHeading(parkingVec, Math.toRadians(0),null,new ProfileAccelConstraint(-20,50));
            driveActions.clear();
            driveActions.add(goPark.build());
        }

        // Update running actions based on current
        List<Action> newDriveActions = new ArrayList<>();
        for (Action action : driveActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newDriveActions.add(action);
            }
        }
        driveActions = newDriveActions;

        if (!driveActions.isEmpty()) {
            // If drive actions is empty (we are going to a position), set rotate to 0 so we don't interfere
            rotate = 0;
        } else if (!manualRotate) {
            // If not manual rotate, let autoLock drive rotate
            rotate = autoLockAngle();
        } else {
            // Otherwise drive rotate with rstick x
            rotate = gamepad1.right_stick_x;
        }

        if (driveActions.isEmpty()) {
            // Drive field relative, but only if we aren't already going to a position
            driveFieldRelative(gamepad1.left_stick_y, -gamepad1.left_stick_x, rotate);
        }

        List<Action> newSystemsActions = new ArrayList<>();
        for (int i = 0; i < systemsActions.size(); i++) {
            if (systemsActions.get(i) == null || !systemsActions.get(i).run(packet)) {
                systemsActions.set(i, null);
            }
        }

        prevGamepad1.copy(gamepad1);
        prevGamepad2.copy(gamepad2);

        PoseStorage.currentPose = drive.localizer.getPose();

        telemetry.addData("Camera has balls", camera.ballDetected());
        telemetry.addData("Position ID", turntable.getPositionId());
        telemetry.addData("Shooter is at speed?", shooter.isAtSpeed());
        telemetry.addData("Current shooter speed", shooter.getVelocity());
        telemetry.addData("Num systems actions", systemsActions.size());
        telemetry.addData("Turntable status", turntable.toString());
        telemetry.addData("DistanceFromGoal",distanceFromGoal);
        telemetry.addData("x", drive.localizer.getPose().position.x);
        telemetry.addData("y", drive.localizer.getPose().position.y);
        telemetry.addData("heading", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
        telemetry.update();

        dash.sendTelemetryPacket(packet); // Send telemetry packet to dash

    }

    @Override
    public void stop() {
        // Stop all servos and set motor power to brake
        drive.leftFront.setPower(0);
        drive.rightFront.setPower(0);
        drive.leftBack.setPower(0);
        drive.rightBack.setPower(0);

        systemsActions.clear();
        driveActions.clear();
    }



    public double autoLockAngle() {
        // Use tangent to calculate the angle needed to face the goal position on the field
        double xDif = autoLockingTarget.x - drive.localizer.getPose().position.x;
        double yDif = autoLockingTarget.y - drive.localizer.getPose().position.y;
        double tolerance = 0.03; // Tolerance in radians

        // Use tan-1 to get raw target heading before normalizing
        double rawTargetHeading = Math.atan2(yDif, xDif);
        // Normalize radians within -360, 360
        // This shouldn't really be necessary as we normalize again later, but it doesn't hurt anything
        rawTargetHeading = AngleUnit.normalizeRadians(rawTargetHeading);
        double realTargetHeading = AngleUnit.normalizeRadians(rawTargetHeading);

        // Deviation is the error between our current heading and the calculated target heading
        double deviation = drive.localizer.getPose().heading.toDouble() - realTargetHeading;
        // Normalize within -360, 360 so it doesn't try to spin multiple times
        deviation = AngleUnit.normalizeRadians(deviation);


        if (Math.abs(deviation) > tolerance) {
            // Only run while error is outside of tolerance
            double kP = 1.0; // Modify to change P strength
            double turnPower = kP * deviation;


            return Math.max(-0.6, Math.min(0.6, turnPower)); // Set max turning speed
        } else {
            // We are aligned, so command no turn.
            return 0.0;
        }
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

    static public double updateDistanceFromGoal(MecanumDrive drive) {
        double xDif = (-72) - drive.localizer.getPose().position.x;
        double yDif =  (72 * PoseStorage.isRed) - drive.localizer.getPose().position.y;
        return Math.sqrt(Math.pow(xDif, 2) + Math.pow(yDif, 2));
    }



}
