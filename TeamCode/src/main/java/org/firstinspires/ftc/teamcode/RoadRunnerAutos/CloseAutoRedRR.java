package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "Close Auto Red - RR", group = "Autonomous")
public class CloseAutoRedRR extends LinearOpMode {

    public double timeBeforeStart = 2.0;

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


        Pose2d initialPose = new Pose2d(-56.245, 52.4553, Math.toRadians(131.0884));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Vector2d pos1 = new Vector2d(-30.9436, 24.6258);
        Vector2d pos2 = new Vector2d(-59.6704, 25.7748);

        TrajectoryActionBuilder gotoAprilRead = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(pos1, Math.toRadians(-152.0563));

        TrajectoryActionBuilder turnToShoot = drive.actionBuilder(new Pose2d(pos1, Math.toRadians(-152.0563)))
                .turnTo(Math.toRadians(131.0084));

        TrajectoryActionBuilder leave = drive.actionBuilder(new Pose2d(pos1, Math.toRadians(145
                )))
                .strafeToLinearHeading(pos2, Math.toRadians(180));

        Shooter shooter = new Shooter(hardwareMap);
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SleepAction(timeBeforeStart));



        Actions.runBlocking(
                new SequentialAction(
                        gotoAprilRead.build(),
                        new SleepAction(1.5)
                ));

        int shotsToCycle = shooter.findShotsToCycle();
        for (int i = 0; i < shotsToCycle; i++) {
            Actions.runBlocking(
                    new SequentialAction(
                            shooter.spinUp(480),
                            new SleepAction(0.5),
                            shooter.fireBall()
                    )
            );
        }

        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(1),
                        turnToShoot.build(),
                        shooter.spinUp(1500),
                        new SleepAction(.5),
                        shooter.fireBall(),
                        shooter.spinUp(1500),
                        new SleepAction(.5),
                        shooter.fireBall(),
                        shooter.spinUp(1500),
                        new SleepAction(.5),
                        shooter.fireBall(),
                        new SleepAction(.25),
                        leave.build(),
                        shooter.stopSpin()
                )
        );

        drive.updatePoseEstimate();
        PoseStorage.currentPose = drive.localizer.getPose();
    }
}
