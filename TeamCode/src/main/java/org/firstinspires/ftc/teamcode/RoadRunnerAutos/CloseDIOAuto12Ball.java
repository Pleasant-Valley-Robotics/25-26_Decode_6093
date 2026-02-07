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
@Autonomous(name = "Close Dio Auto 12 Ball", group = "Autonomous")
public class CloseDIOAuto12Ball extends LinearOpMode {

    public double timeBeforeStart = 0.0;
    private MecanumDrive drive = null;
    private int r = PoseStorage.isRed;


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


        Pose2d initialPose = new Pose2d(-59.91, 56.13 * r, Math.toRadians(-128.87));


        Vector2d shootPosition = new Vector2d(-23.9816, 14.2421 * r);
        Vector2d middleSpike = new Vector2d(14.4952, 36.4343  * r);
        Vector2d closeSpike = new Vector2d(-13.2232, 36.4343  * r);
        Vector2d farSpike = new Vector2d(-41.6725, 36.4343 * r);
        Vector2d gate = new Vector2d(3.5728, 57.0165 * r);


        drive = new MecanumDrive(hardwareMap, initialPose);
        Turntable turntable = new Turntable(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Camera camera = new Camera(hardwareMap);


        TrajectoryActionBuilder moveBack = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(shootPosition, Math.toRadians(131 * r));

        TrajectoryActionBuilder gotoClose = drive.actionBuilder(new Pose2d(shootPosition, Math.toRadians(-131)))
                .strafeToLinearHeading(closeSpike, Math.toRadians(90 * r));

        TrajectoryActionBuilder gotoMiddle = drive.actionBuilder(new Pose2d(shootPosition, Math.toRadians(-131)))
                .strafeToLinearHeading(middleSpike, Math.toRadians(90 * r));

        TrajectoryActionBuilder gotoFar = drive.actionBuilder(new Pose2d(shootPosition, Math.toRadians(-131)))
                .strafeToLinearHeading(farSpike, Math.toRadians(90 * r));


        turntable.addBall(1, Turntable.IndexColors.PURPLE);
        turntable.addBall(3, Turntable.IndexColors.PURPLE);
        turntable.addBall(5, Turntable.IndexColors.PURPLE);


        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SleepAction(timeBeforeStart));

        turntable.updatePosition();

        shooter.spinUp(1320);

        Actions.runBlocking(moveBack.build());


        //int shotsToCycle = camera.findShotsToCycle();


        while (!shooter.isAtSpeed()) {
            Actions.runBlocking(new SleepAction(0.2));
        }
        Actions.runBlocking(
                new SequentialAction(
                        shooter.shootAllFAST(turntable)
                ));

        shooter.stop();
        intake.setPower(1);
        turntable.updatePosition();

        Actions.runBlocking(
            new ParallelAction(
                intake.normalIntake(camera, turntable),
                new SequentialAction(
//                    drive.actionBuilder(drive.localizer.getPose()).strafeToLinearHeading(middleSpike, Math.toRadians(-90)).build(),
//                    new SleepAction(0.2),
//                    drive.actionBuilder(new Pose2d(middleSpike, Math.toRadians(-90)))
//                        .strafeToLinearHeading(new Vector2d(middleSpike.x, middleSpike.y - 5), Math.toRadians(-90)).build(),
//                    new SleepAction(0.2),
//                    drive.actionBuilder(new Pose2d(middleSpike.x, middleSpike.y - 5, Math.toRadians(-90)))
//                        .strafeToLinearHeading(new Vector2d(middleSpike.x, middleSpike.y - 10), Math.toRadians(-90)).build(),
//                    new SleepAction(0.2)
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(middleSpike, Math.toRadians(90))
                            .strafeToLinearHeading(new Vector2d(middleSpike.x, middleSpike.y + (15 * r)), Math.toRadians(90 * r), new TranslationalVelConstraint(8.0)).build()


                )
        ));



        intake.stopIntake();

        shooter.spinUp(1300);
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(gate.x, gate.y + (3 * r)), Math.toRadians(90 * r))
                .strafeToLinearHeading(new Vector2d(gate.x, gate.y + (30 * r)), Math.toRadians(165 * r) , null, new ProfileAccelConstraint(-40, 60)).build());

        for (int i = 0; i < 10; i++) {
            if (camera.findShotsToCycle() != -1) {
                PoseStorage.shotsToCycle = camera.findShotsToCycle();
            }
        }

        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose()).strafeToLinearHeading(shootPosition, Math.toRadians(131 * r)).build());
        turntable.turnToPosition(1);
        Actions.runBlocking(shooter.shootInPattern(turntable));

        shooter.stop();
        intake.stopIntake();

        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(1)
                ));


        drive.updatePoseEstimate();
        PoseStorage.currentPose = drive.localizer.getPose();
        //PoseStorage.shotsToCycle = shotsToCycle;
    }
}
