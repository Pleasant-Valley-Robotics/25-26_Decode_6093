package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Config
@Autonomous(name = "Test Aut2o", group = "Autonomous")
public class TestAuto2 extends LinearOpMode {
    Turntable turntable;
    Shooter shooter;
    Intake intake;
    Camera camera;
    MecanumDrive drive;

    public static double driveSpeed = 0.13;


    public void runOpMode() {

        turntable = new Turntable(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        camera = new Camera(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        turntable.updatePosition();

        waitForStart();
        Actions.runBlocking(new SleepAction(2));
        intake.setPower(1);

        intakeBalls(5.0);

        intake.setPower(-1);
        Actions.runBlocking(new SleepAction(0.5));
    }

    private void intakeBalls(double limitTime) {
        intake.setPower(1);
        drive.rightBack.setPower(driveSpeed);
        drive.rightFront.setPower(driveSpeed);
        drive.leftBack.setPower(driveSpeed);
        drive.leftFront.setPower(driveSpeed);

        Actions.runBlocking(
                new SequentialAction(
                        intake.autoIntake(camera, turntable, limitTime),
                        intake.reverse()
                )
        );

        drive.rightBack.setPower(0);
        drive.rightFront.setPower(0);
        drive.leftBack.setPower(0);
        drive.leftFront.setPower(0);

        drive.updatePoseEstimate();
    }
}
