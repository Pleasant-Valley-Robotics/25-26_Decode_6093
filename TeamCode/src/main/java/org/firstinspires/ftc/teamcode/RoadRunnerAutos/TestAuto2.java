package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Test Aut2o", group = "Autonomous")
public class TestAuto2 extends LinearOpMode {
    public void runOpMode() {
        Turntable turntable = new Turntable(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Camera camera = new Camera(hardwareMap);

        turntable.updatePosition();

        waitForStart();
        Actions.runBlocking(new SleepAction(2));
        intake.setPower(1);
        Actions.runBlocking(intake.normalIntake(camera, turntable));

        intake.setPower(-1);
        Actions.runBlocking(new SleepAction(0.5));
    }
}
