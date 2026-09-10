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

// TODO:
// Maybe: add camera auto locking
// Find a way to let g2 control launch speed
// Slow mode
// Add an option for field centric

// LED doesn't work
// Flicker doesn't work



@TeleOp(name="NOT ODO DIO", group="aaaOdometry")
@Config
public class DioDriveController extends OpMode {
    private DcMotor fr;
    private DcMotor fl;
    private DcMotor br;
    private DcMotor bl;

    private Turntable turntable;
    private Intake intake;
    private Shooter shooter;

    private final int flyWheelSpeed = 1330;


    public void init() {

        // Instantiate our basic part classes
        turntable = new Turntable(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        // Tell the hardware map to find our drive motors
        fr = hardwareMap.get(DcMotor.class, "frontRightDrive");
        fl = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        br = hardwareMap.get(DcMotor.class, "backRightDrive");
        bl = hardwareMap.get(DcMotor.class, "backLeftDrive");

        // Reverse half of the motors to simplify controls
        fr.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);

        // All motors are set to stop at zero power, rather than coasting
        fr.setZeroPowerBehavior(BRAKE);
        fl.setZeroPowerBehavior(BRAKE);
        br.setZeroPowerBehavior(BRAKE);
        bl.setZeroPowerBehavior(BRAKE);
    }

    public void loop() {
        // Use g1 to mecanum drive the robot. Pressing right bumper makes it slower
        if (gamepad1.right_bumper) {
            calculateMotorPower(0.25);
        } else {
            calculateMotorPower(.5);
        }

        // Control intake power using g2 left stick y
        intake.setPower(gamepad2.left_stick_y);

        // Buttons B, Y, and X tell the turntable to move to a predetermined position
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

        // Pressing the right bumper gives the turntable a few extra degrees
        // of rotation. This allows a ball to slot into place if it gets stuck
        if (gamepad2.right_bumper) turntable.extraRange(false);
        else if (gamepad2.rightBumperWasReleased()) turntable.extraRange(true);

        // Right and left trigger to stop and start shooter motor
        if (gamepad2.right_trigger > 0) shooter.spinUp(flyWheelSpeed);
        if (gamepad2.left_trigger > 0) shooter.stop();


        // This code should really be in the shooter class
        if (shooter.isAtSpeed() && shooter.isMoving()) {
            shooter.setLedIntensity(0.5);
        } else {
            shooter.setLedIntensity(0);
        }

        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("Position ID", turntable.getPositionId());
        telemetry.addData("Target speed", flyWheelSpeed);
        telemetry.addData("Shooter is at speed?", shooter.isAtSpeed());
        telemetry.addData("Current shooter speed", shooter.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }

    private void calculateMotorPower(double maxDriveSpeed) {
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

        fr.setPower(frPower * maxDriveSpeed);
        fl.setPower(flPower * maxDriveSpeed);
        br.setPower(brPower * maxDriveSpeed);
        bl.setPower(blPower * maxDriveSpeed);
    }

}
