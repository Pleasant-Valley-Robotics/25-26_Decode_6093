package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RoadRunnerAutos.Camera;
import org.firstinspires.ftc.teamcode.RoadRunnerAutos.Intake;
import org.firstinspires.ftc.teamcode.RoadRunnerAutos.PoseStorage;
import org.firstinspires.ftc.teamcode.RoadRunnerAutos.Shooter;
import org.firstinspires.ftc.teamcode.RoadRunnerAutos.Turntable;

import java.util.ArrayList;
import java.util.List;
@TeleOp(name="Distance Tuner", group="Odometry")
public class DistanceTuner extends OpMode {
    private MecanumDrive drive;
    Turntable turntable;
    Intake intake;
    Shooter shooter;

    Vector2d autoLockingTarget = new Vector2d(-72, 72 * PoseStorage.isRed);
    private int flyWheelSpeed = 10;
    private double distanceFromGoal;

    @Override
    public void init() {
        // Declare all systems so they can be used in loop
        // This has to be done here because hardwareMap doesn't exist until startup
        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose); // Pass saved pose from autos
        turntable = new Turntable(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
    }

    @Override
    public void loop() {
        drive.updatePoseEstimate(); // Get pose from odometry
        distanceFromGoal = updateDistanceFromGoal(drive);

        intake.setPower(gamepad2.left_stick_y);


        if (gamepad1.dpadLeftWasPressed())
            drive.localizer.setPose(new Pose2d(0, 0, Math.toRadians(90 * PoseStorage.isRed)));

        if (gamepad2.xWasPressed()) turntable.turnToPosition(0);
        if (gamepad2.yWasPressed()) turntable.turnToPosition(1);
        if (gamepad2.bWasPressed()) turntable.turnToPosition(2);

        if (gamepad2.dpadRightWasPressed()) {
            flyWheelSpeed += 10;
        }

        if (gamepad2.dpadLeftWasPressed()) {
            flyWheelSpeed -= 10;
        }

        if (gamepad2.right_trigger > 0) shooter.spinUp(flyWheelSpeed);
        if (gamepad2.left_trigger > 0) shooter.stop();
        if (gamepad2.dpadUpWasPressed()) shooter.setServoPos(shooter.upPos);
        if (gamepad2.dpadDownWasPressed()) shooter.setServoPos(shooter.downPos);

        if (shooter.getLastSpeed() != 0) {
            shooter.spinUp(flyWheelSpeed);
        }

        if (shooter.isAtSpeed() && shooter.isMoving()) {
            gamepad2.rumble(1);
        }

        PoseStorage.currentPose = drive.localizer.getPose();

        telemetry.addData("Target speed", flyWheelSpeed);
        telemetry.addData("Shooter is at speed?", shooter.isAtSpeed());
        telemetry.addData("Current shooter speed", shooter.getVelocity());
        telemetry.addData("Turntable status", turntable.toString());
        telemetry.addData("DistanceFromGoal",distanceFromGoal);
        telemetry.addData("x", drive.localizer.getPose().position.x);
        telemetry.addData("y", drive.localizer.getPose().position.y);
        telemetry.addData("heading", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
        telemetry.update();
    }

    static public double updateDistanceFromGoal(MecanumDrive drive) {
        double xDif = (-72) - drive.localizer.getPose().position.x;
        double yDif =  (72 * PoseStorage.isRed) - drive.localizer.getPose().position.y;
        return Math.sqrt(Math.pow(xDif, 2) + Math.pow(yDif, 2));
    }

}
