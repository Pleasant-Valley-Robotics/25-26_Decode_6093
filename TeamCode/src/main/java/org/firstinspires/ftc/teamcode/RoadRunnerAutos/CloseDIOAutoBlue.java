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
@Autonomous(name = "Close Dio Auto Blue", group = "Autonomous")
public class CloseDIOAutoBlue extends LinearOpMode {

    public double timeBeforeStart = 0.0;


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
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Turntable turntable = new Turntable(hardwareMap);
        Camera camera = new Camera(hardwareMap);

        Vector2d pos1 = new Vector2d(-25.84, -17.39);

        TrajectoryActionBuilder moveBack = drive.actionBuilder(initialPose)
                .strafeToSplineHeading(pos1, Math.toRadians(-135));

        TrajectoryActionBuilder turnToShoot = drive.actionBuilder(new Pose2d(pos1, Math.toRadians(152.0563)))
                .turnTo(Math.toRadians(-131.0084));


        Shooter shooter = new Shooter(hardwareMap);
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SleepAction(timeBeforeStart));



        Actions.runBlocking(
                new SequentialAction(
                        moveBack.build()
                ));


        //int shotsToCycle = camera.findShotsToCycle();

        shooter.spinUp(1200);

        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(5),
                        shooter.shootAll(turntable)
                ));

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
