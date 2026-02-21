package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
        Turntable turntable = new Turntable(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Camera camera = new Camera(hardwareMap);
        turntable.updatePosition();

        waitForStart();

        intake.setPower(1);
        Actions.runBlocking(intake.normalIntake(camera, turntable));
        intake.setPower(-1);

    }
}
