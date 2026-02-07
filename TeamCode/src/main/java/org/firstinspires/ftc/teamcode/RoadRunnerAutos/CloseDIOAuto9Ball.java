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
    private int r = 1;


    @Override
    public void runOpMode() {

        while (!isStopRequested() && !opModeIsActive()) {

            if (gamepad1.dpadUpWasPressed()) {
                timeBeforeStart += 1.0;
            }
            if (gamepad1.dpadDownWasPressed()) {
                timeBeforeStart -= 1.0;
            }

            if (gamepad1.aWasPressed()) {
                PoseStorage.isRed = 1;
            }

            if (gamepad1.bWasPressed()) {
                PoseStorage.isRed = -1;
            }

            if (r == 1) {
                telemetry.addLine("Red");
            } else {
                telemetry.addLine("Blue");
            }
            r = PoseStorage.isRed;
            telemetry.addData("Wait Time", timeBeforeStart);
            telemetry.update();
        }


        Pose2d initialPose = new Pose2d(-59.91, 56.13 * r, Math.toRadians(128.87 * r));


        Vector2d shootPosition = new Vector2d(-23.9816, 14.2421*r); // 130.5
        Vector2d leaveShoot = new Vector2d(-34.358,15.02*r); // 126.0861
        Vector2d middleSpike = new Vector2d(15.0561-(3.52*r), 32.7851*r);
        Vector2d closeSpike = new Vector2d(-15.2232-(3.52*r), 34.8*r);
        Vector2d farSpike = new Vector2d(41.6725-(2*r), 35.8*r);
        Vector2d gate = new Vector2d(3.5728, 58.0165*r);
        double shootAngle = 130.5*r;
        double leaveShootAngle = 126.0861*r;
        double intakeAngle = 90*r;


        drive = new MecanumDrive(hardwareMap, initialPose);
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


        shooter.spinUp(1320);

        //Drive to shoot
        Actions.runBlocking(drive.actionBuilder(initialPose)
                .strafeToLinearHeading(shootPosition, Math.toRadians(-180 * r),null,new ProfileAccelConstraint(-30,70)).build());


        // Read motif
        PoseStorage.shotsToCycle = camera.findShotsToCycle();

        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(shootPosition, Math.toRadians(shootAngle)).build());


        // Shoot
        while (!shooter.isAtSpeed()) {
            Actions.runBlocking(new SleepAction(0.1));
        }
        Actions.runBlocking(shooter.shootInPattern(turntable));

        shooter.stop();



        // Intake
        intake.setPower(1);

        Actions.runBlocking(
            new ParallelAction(
                intake.normalIntake(camera, turntable),
                new SequentialAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                .strafeToLinearHeading(closeSpike, Math.toRadians(intakeAngle),null,new ProfileAccelConstraint(-30,70))
                                .strafeToLinearHeading(new Vector2d(closeSpike.x, closeSpike.y + (13*r)), Math.toRadians(intakeAngle), new TranslationalVelConstraint(4.2),new ProfileAccelConstraint(-30,70)).build()),
                    updatePose(new Pose2d(closeSpike.x, closeSpike.y + (14*r), Math.toRadians(intakeAngle)))
                )
        );

        intake.setPower(-1);


        shooter.spinUp(1320);

        // Drive to shoot
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose()).strafeToLinearHeading(shootPosition, Math.toRadians(shootAngle)).build());
        turntable.turnToPosition(1);

        // Shoot
        while (!shooter.isAtSpeed()) {
            Actions.runBlocking(new SleepAction(0.1));
        }
        Actions.runBlocking(shooter.shootInPattern(turntable));

        Actions.runBlocking(new SleepAction(0.1));

        shooter.stop();

        // Intake
        intake.setPower(1);

        Actions.runBlocking(
                new ParallelAction(
                        intake.normalIntake(camera, turntable),
                        new SequentialAction(
                                drive.actionBuilder(drive.localizer.getPose())
                                        .strafeToLinearHeading(middleSpike, Math.toRadians(intakeAngle),null,new ProfileAccelConstraint(-30,70))
                                        .strafeToLinearHeading(new Vector2d(middleSpike.x, middleSpike.y + (14*r)), Math.toRadians(intakeAngle), new TranslationalVelConstraint(4.2), new ProfileAccelConstraint(-30,70)).build(),
                                updatePose(new Pose2d(middleSpike.x, middleSpike.y + (14*r), Math.toRadians(intakeAngle)))
                        )
                ));

        intake.setPower(-1);

        shooter.spinUp(1320);

        // Drive to shoot
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose()).strafeToLinearHeading(leaveShoot, Math.toRadians(leaveShootAngle)).build());
        turntable.turnToPosition(1);

        // Shoot
        while (!shooter.isAtSpeed()) {
            Actions.runBlocking(new SleepAction(0.1));
        }
        Actions.runBlocking(shooter.shootInPattern(turntable));

        Actions.runBlocking(new SleepAction(0.1));

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
