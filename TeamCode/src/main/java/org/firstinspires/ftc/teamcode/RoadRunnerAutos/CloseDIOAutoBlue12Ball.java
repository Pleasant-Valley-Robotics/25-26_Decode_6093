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
@Autonomous(name = "Close Dio Auto Blue 12 Ball", group = "Autonomous")
public class CloseDIOAutoBlue12Ball extends LinearOpMode {

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


        Vector2d shootPosition = new Vector2d(-23.9816, -14.2421);
        Vector2d middleSpike = new Vector2d(14.4952, -36.4343);
        Vector2d closeSpike = new Vector2d(-13.2232, -36.4343);
        Vector2d farSpike = new Vector2d(-41.6725, -36.4343);
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
        turntable.addBall(5, Turntable.IndexColors.PURPLE);


        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SleepAction(timeBeforeStart));

        turntable.updatePosition();

        shooter.spinUp(1300);

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
                intake.autoIntakeFAST(camera, turntable),
                new SequentialAction(
                    drive.actionBuilder(drive.localizer.getPose()).strafeToLinearHeading(middleSpike, Math.toRadians(-90)).build(),
                    new SleepAction(0.2),
                    drive.actionBuilder(new Pose2d(middleSpike, Math.toRadians(-90)))
                        .strafeToLinearHeading(new Vector2d(middleSpike.x, middleSpike.y - 5), Math.toRadians(-90)).build(),
                    new SleepAction(0.2),
                    drive.actionBuilder(new Pose2d(middleSpike.x, middleSpike.y - 5, Math.toRadians(-90)))
                        .strafeToLinearHeading(new Vector2d(middleSpike.x, middleSpike.y - 10), Math.toRadians(-90)).build(),
                    new SleepAction(0.2)
//                    drive.actionBuilder(drive.localizer.getPose())
//                            .strafeToLinearHeading(middleSpike, Math.toRadians(-90))
//                            .strafeToLinearHeading(new Vector2d(middleSpike.x, middleSpike.y - 15), Math.toRadians(-90), new TranslationalVelConstraint(8.0)).build()


                )
        ));



        intake.stopIntake();

        shooter.spinUp(1300);
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(gate.x, gate.y - 3), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(gate.x, gate.y + 30), Math.toRadians(165) , null, new ProfileAccelConstraint(-40, 60)).build());

        for (int i = 0; i < 10; i++) {
            if (camera.findShotsToCycle() != -1) {
                PoseStorage.shotsToCycle = camera.findShotsToCycle();
            }
        }

        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose()).strafeToLinearHeading(shootPosition, Math.toRadians(-131)).build());
        turntable.turnToPosition(0);
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
        PoseStorage.isRed = -1;
    }
}
