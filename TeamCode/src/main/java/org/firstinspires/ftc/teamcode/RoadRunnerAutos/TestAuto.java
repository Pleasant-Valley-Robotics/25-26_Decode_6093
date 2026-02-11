package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Autonomous(name = "Test Auto", group = "Autonomous")
public class TestAuto extends LinearOpMode {
    private MecanumDrive drive;
    private Turntable turntable;
    private Shooter shooter;
    private Camera camera;
    private Intake intake;
    private boolean isFinished = false;

    @Override
    public void runOpMode() {
        PoseStorage.isRed = -1;
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        turntable = new Turntable(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        camera = new Camera(hardwareMap);

        waitForStart();
        turntable.updatePosition();
        intake.setPower(1);
        Actions.runBlocking(
            intake.normalIntake(camera, turntable)
        );
        intake.setPower(0);

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                drive.actionBuilder(new Pose2d(0, 0, 0)).lineToX(33).build(),
                                stopUpdate()
                        ),
                        updatePose()
                )

        );

        drive.localizer.update();
        PoseStorage.currentPose = drive.localizer.getPose();

    }

    private Action updatePose() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                drive.localizer.update();
                PoseStorage.currentPose = drive.localizer.getPose();

                if (isFinished) {
                    isFinished = false;
                    return false;
                }

                return true;
            }
        };
    }

    public Action stopUpdate() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                isFinished = true;
                return false;
            }
        };
    }



}
