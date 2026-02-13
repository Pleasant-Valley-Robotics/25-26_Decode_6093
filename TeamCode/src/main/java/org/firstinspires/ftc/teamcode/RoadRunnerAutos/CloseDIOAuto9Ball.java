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
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "Close Dio Auto 9 Ball", group = "Autonomous")
public class CloseDIOAuto9Ball extends LinearOpMode {
    public double timeBeforeStart = 0.0;
    private MecanumDrive drive = null;


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

        double shootAngle = 134.5*PoseStorage.isRed;
        double leaveShootAngle = 127.4*PoseStorage.isRed;
        double intakeAngle = 90*PoseStorage.isRed;


        drive = new MecanumDrive(hardwareMap, new Pose2d(Positions.getCloseStartPose(), Math.toRadians(131.9529) * PoseStorage.isRed));
        Turntable turntable = new Turntable(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Camera camera = new Camera(hardwareMap);


        turntable.addBall(0, Turntable.IndexColors.PURPLE);
        turntable.addBall(1, Turntable.IndexColors.GREEN);
        turntable.addBall(2, Turntable.IndexColors.PURPLE);




        waitForStart();

        if (isStopRequested()) return;

        turntable.updatePosition();

        Actions.runBlocking(new SleepAction(timeBeforeStart));


        shooter.spinUp(1360);

        //Drive to shoot
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(Positions.getShootPose(), Math.toRadians(-180 * PoseStorage.isRed),null,new ProfileAccelConstraint(-30,70)).build());


        // Read motif
        PoseStorage.shotsToCycle = camera.findShotsToCycle();

        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(Positions.getShootPose(), Math.toRadians(shootAngle)).build());


        // Shoot
        intake.setPower(-1);
        Actions.runBlocking(new SleepAction(0.05));
        while (!shooter.isAtSpeed()) {
            Actions.runBlocking(new SleepAction(0.1));
        }

        intake.setPower(1);
        Actions.runBlocking(shooter.shootInPattern(turntable));
        intake.stopIntake();
        shooter.stop();



        // Intake
        intake.setPower(1);

        Actions.runBlocking(
            new ParallelAction(
                intake.autoIntake(camera, turntable),
                new SequentialAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                .strafeToLinearHeading(Positions.getIntakeClosePoseS(), Math.toRadians(intakeAngle),null,new ProfileAccelConstraint(-30,70))
                                .strafeToLinearHeading(Positions.getIntakeClosePoseE(), Math.toRadians(intakeAngle), new TranslationalVelConstraint(4.6),new ProfileAccelConstraint(-30,70)).build())
                )
        );


        shooter.spinUp(1360);

        // Drive to shoot
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose()).strafeToLinearHeading(Positions.getLeaveShootPose(), Math.toRadians(leaveShootAngle)).build());

        // Shoot
        intake.setPower(-1);
        Actions.runBlocking(new SleepAction(0.05));
        while (!shooter.isAtSpeed()) {
            Actions.runBlocking(new SleepAction(0.1));
        }

        intake.setPower(1);
        Actions.runBlocking(shooter.shootInPattern(turntable));
        intake.stopIntake();
        shooter.stop();

        // Intake
        intake.setPower(1);

        Actions.runBlocking(
                new ParallelAction(
                        intake.autoIntake(camera, turntable),
                        new SequentialAction(
                                drive.actionBuilder(drive.localizer.getPose())
                                        .strafeToLinearHeading(Positions.getIntakeMedPoseS(), Math.toRadians(intakeAngle),null,new ProfileAccelConstraint(-30,70))
                                        .strafeToLinearHeading(Positions.getIntakeMedPoseE(), Math.toRadians(intakeAngle), new TranslationalVelConstraint(4.6), new ProfileAccelConstraint(-30,70)).build()
                        )
                ));

        shooter.spinUp(1360);

        // Drive to shoot
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose()).strafeToLinearHeading(Positions.getLeaveShootPose(), Math.toRadians(leaveShootAngle)).build());

        intake.setPower(-1);
        Actions.runBlocking(new SleepAction(0.05));
        while (!shooter.isAtSpeed()) {
            Actions.runBlocking(new SleepAction(0.1));
        }

        intake.setPower(1);
        Actions.runBlocking(shooter.shootInPattern(turntable));
        intake.stopIntake();
        shooter.stop();

        drive.updatePoseEstimate();
        PoseStorage.currentPose = drive.localizer.getPose();
        //PoseStorage.shotsToCycle = shotsToCycle;


        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(1)
                ));
    }

    private Action updatePose(Pose2d position) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                drive.updatePoseEstimate();
                PoseStorage.currentPose = position;
                return false;
            }
        };
    }



}
