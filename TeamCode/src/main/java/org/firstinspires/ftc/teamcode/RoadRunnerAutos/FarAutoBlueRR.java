package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunnerAutos.*;

@Config
@Autonomous(name = "Far Auto Blue - RR", group = "Autonomous")
public class FarAutoBlueRR extends LinearOpMode {

    public double timeBeforeStart = 5.0;


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


        Pose2d initialPose = new Pose2d(65.0572, -14.9984, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Vector2d pos1 = new Vector2d(53.2374, -15.8418);
        Vector2d pos2 = new Vector2d(43.4175, -18.4409);

        TrajectoryActionBuilder gotoShoot = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(pos1, Math.toRadians(-163.0));

        TrajectoryActionBuilder leave = drive.actionBuilder(new Pose2d(pos1, Math.toRadians(-163.0)))
                        .strafeToLinearHeading(pos2, Math.toRadians(180));

        Shooter shooter = new Shooter(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SleepAction(timeBeforeStart));

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
                        gotoShoot.build(),
                        shooter.spinUp(1630),
                        new SleepAction(.5),
                        shooter.fireBall(),
                        shooter.spinUp(1630),
                        new SleepAction(.5),
                        shooter.fireBall(),
                        shooter.spinUp(1630),
                        new SleepAction(.5),
                        shooter.fireBall(),
                        new SleepAction(.25),
                        leave.build(),
                        shooter.stopSpin()

                )
        );

        drive.updatePoseEstimate();
        PoseStorage.currentPose = drive.localizer.getPose();
        PoseStorage.isRed = -1;

    }
}