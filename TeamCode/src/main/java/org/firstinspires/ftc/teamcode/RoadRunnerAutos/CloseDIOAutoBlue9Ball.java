package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import com.acmerobotics.dashboard.config.Config;
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
@Autonomous(name = "Close Dio Auto Blue 9 Ball", group = "Autonomous")
public class CloseDIOAutoBlue9Ball extends LinearOpMode {

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
            telemetry.addData("Wait Time", timeBeforeStart);
        }


        Pose2d initialPose = new Pose2d(-59.91, -56.13, Math.toRadians(-128.87));


        Vector2d shootPosition = new Vector2d(-23.9816, -14.2421); // -130
        Vector2d leaveShoot = new Vector2d(-30.3306,-9.5704); // -126.98
        Vector2d middleSpike = new Vector2d(14.4952, -35);
        Vector2d closeSpike = new Vector2d(-13.2232, -35.8);
        Vector2d farSpike = new Vector2d(-41.6725, -35.8);
        Vector2d gate = new Vector2d(3.5728, -57.0165);


        drive = new MecanumDrive(hardwareMap, initialPose);
        Turntable turntable = new Turntable(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Camera camera = new Camera(hardwareMap);


        TrajectoryActionBuilder moveBack = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(shootPosition, Math.toRadians(-131));

        TrajectoryActionBuilder gotoClose = drive.actionBuilder(new Pose2d(shootPosition, Math.toRadians(-131)))
                .strafeToLinearHeading(closeSpike, Math.toRadians(-90));

        TrajectoryActionBuilder gotoMiddle = drive.actionBuilder(new Pose2d(shootPosition, Math.toRadians(-131)))
                .strafeToLinearHeading(middleSpike, Math.toRadians(-90));

        TrajectoryActionBuilder gotoFar = drive.actionBuilder(new Pose2d(shootPosition, Math.toRadians(-131)))
                .strafeToLinearHeading(farSpike, Math.toRadians(-90));


        turntable.addBall(1, Turntable.IndexColors.PURPLE);
        turntable.addBall(3, Turntable.IndexColors.PURPLE);
        turntable.addBall(5, Turntable.IndexColors.GREEN);




        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SleepAction(timeBeforeStart));

        turntable.updatePosition();

        shooter.spinUp(1300);

        //Drive to shoot
        Actions.runBlocking(drive.actionBuilder(initialPose)
                .strafeToLinearHeading(shootPosition, Math.toRadians(-180),null,new ProfileAccelConstraint(-30,70)).build());


        // Read motif
        while (camera.findShotsToCycle() == -1) {
            new SleepAction(0.1);
        }
        PoseStorage.shotsToCycle = camera.findShotsToCycle();



        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(shootPosition, Math.toRadians(-130)).build());


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
                intake.autoIntake(camera, turntable),
                new SequentialAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                .strafeToLinearHeading(closeSpike, Math.toRadians(-90),null,new ProfileAccelConstraint(-30,70))
                                .strafeToLinearHeading(new Vector2d(closeSpike.x, closeSpike.y - 13), Math.toRadians(-90), new TranslationalVelConstraint(4),new ProfileAccelConstraint(-30,70)).build())
                )
        );

        intake.stopIntake();


        shooter.spinUp(1300);

        // Drive to shoot
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose()).strafeToLinearHeading(shootPosition, Math.toRadians(-130)).build());
        turntable.turnToPosition(0);

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
                        intake.autoIntake(camera, turntable),
                        new SequentialAction(
                                drive.actionBuilder(drive.localizer.getPose())
                                        .strafeToLinearHeading(middleSpike, Math.toRadians(-90),new TranslationalVelConstraint(60))
                                        .strafeToLinearHeading(new Vector2d(middleSpike.x, middleSpike.y - 13.5), Math.toRadians(-90), new TranslationalVelConstraint(4), new ProfileAccelConstraint(-50,50)).build()
                        )
                ));

        intake.stopIntake();

        shooter.spinUp(1300);

        // Drive to shoot
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose()).strafeToLinearHeading(leaveShoot, Math.toRadians(-127)).build());
        turntable.turnToPosition(0);

        // Shoot
        while (!shooter.isAtSpeed()) {
            Actions.runBlocking(new SleepAction(0.1));
        }
        Actions.runBlocking(shooter.shootInPattern(turntable));

        Actions.runBlocking(new SleepAction(0.1));

        shooter.stop();

        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(1)
                ));


        drive.updatePoseEstimate();
        PoseStorage.currentPose = drive.localizer.getPose();
        //PoseStorage.shotsToCycle = shotsToCycle;
        PoseStorage.isRed = -1;
    }
}
