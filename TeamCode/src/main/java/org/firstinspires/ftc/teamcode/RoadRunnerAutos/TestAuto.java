package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.CancelableProfile;
import com.acmerobotics.roadrunner.CompositeVelConstraint;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "Test Auto", group = "Autonomous")
public class TestAuto extends LinearOpMode {
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Turntable turntable = new Turntable(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Camera camera = new Camera(hardwareMap);

        turntable.updatePosition();

        waitForStart();
        TrajectoryActionBuilder driveForward =  drive.actionBuilder(drive.localizer.getPose()).lineToX(20, new TranslationalVelConstraint(4.5));


        intake.setPower(1);
        Actions.runBlocking(
                new RaceAction(
                    new SequentialAction(
                            driveForward.build()
                    ),
                    new SequentialAction(
                            intake.autoIntake(camera, turntable),
                            intake.reverse(),
                            drive.actionBuilder(drive.localizer.getPose()).strafeTo(new Vector2d(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y)).build()
                        )
                )
            
        );


        stopDriving(drive);

        intake.stopIntake();
        Actions.runBlocking(new SleepAction(2));
    }

    public void stopDriving(MecanumDrive drive) {
        drive.rightBack.setPower(0);
        drive.rightFront.setPower(0);
        drive.leftBack.setPower(0);
        drive.leftFront.setPower(0);
    }
}
