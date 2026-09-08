package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RoadRunnerAutos.Camera;
import org.firstinspires.ftc.teamcode.RoadRunnerAutos.Intake;
import org.firstinspires.ftc.teamcode.RoadRunnerAutos.Shooter;
import org.firstinspires.ftc.teamcode.RoadRunnerAutos.Turntable;

@TeleOp(name="ODODIO", group="aaaOdometry")
@Config
public class DioDriveController extends OpMode {
    private DcMotor fr;
    private DcMotor fl;
    private DcMotor br;
    private DcMotor bl;

    private Turntable turntable;
    private Intake intake;
    private Camera camera;
    private Shooter shooter;


    public void init() {

        turntable = new Turntable(hardwareMap);
        intake = new Intake(hardwareMap);
        camera = new Camera(hardwareMap);
        shooter = new Shooter(hardwareMap);


        fr = hardwareMap.get(DcMotor.class, "frontRightDrive");
        fl = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        br = hardwareMap.get(DcMotor.class, "backRightDrive");
        bl = hardwareMap.get(DcMotor.class, "backLeftDrive");


        fr.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);

        fr.setZeroPowerBehavior(BRAKE);
        fl.setZeroPowerBehavior(BRAKE);
        br.setZeroPowerBehavior(BRAKE);
        bl.setZeroPowerBehavior(BRAKE);
    }

    public void loop() {
        calculateMotorPower();

        intake.setPower(gamepad2.left_stick_y);

        if (gamepad2.bWasPressed()) {
            turntable.turnToPosition(0);
            shooter.setServoPos(shooter.downPos);
        }
        if (gamepad2.yWasPressed()) {
            turntable.turnToPosition(1);
            shooter.setServoPos(shooter.downPos);
        }
        if (gamepad2.xWasPressed()) {
            turntable.turnToPosition(2);
            shooter.setServoPos(shooter.downPos);
        }

        if (gamepad2.right_bumper) turntable.extraRange(false);
        else if (gamepad2.rightBumperWasReleased()) turntable.extraRange(true);

        

    }

    private void calculateMotorPower() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;

        double rotate = gamepad1.right_stick_x;

        double frPower = forward - strafe - rotate;
        double flPower = forward + strafe + rotate;
        double brPower = forward + strafe - rotate;
        double blPower = forward - strafe + rotate;

        double max = Math.max(Math.abs(frPower), Math.max(Math.abs(flPower), Math.max(Math.abs(blPower), Math.abs(brPower))));

        if (max > 1.0) {
            frPower /= max;
            flPower /= max;
            blPower /= max;
            brPower /= max;
        }

        fr.setPower(frPower);
        fl.setPower(flPower);
        br.setPower(brPower);
        bl.setPower(blPower);
    }

}
