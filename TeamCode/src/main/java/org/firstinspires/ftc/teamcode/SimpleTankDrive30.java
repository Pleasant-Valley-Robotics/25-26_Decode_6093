package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Simple Tank Drive - 30%", group="Linear Opmode")
public class SimpleTankDrive30 extends LinearOpMode {

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    @Override
    public void runOpMode() {
        // Map the motors (make sure these names match the Driver Station config!)
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Reverse one side so forward stick makes the robot go forward
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start
        waitForStart();

        while (opModeIsActive()) {
            // Tank drive: left stick = forward/back, right stick X = turn
            double driveSpeed = 0.30;
            double turnSpeed = 0.25;

            double drive = -gamepad1.left_stick_y * driveSpeed;   // forward/back
            double turn  =  gamepad1.right_stick_x * turnSpeed;  // rotation

            // Mix drive and turn for each side
            double leftPower  = drive + turn;
            double rightPower = drive - turn;

            // Clip values to stay within -1 and 1
            leftPower  = Math.max(-1, Math.min(1, leftPower));
            rightPower = Math.max(-1, Math.min(1, rightPower));

            // Send power to motors
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // Telemetry
            telemetry.addData("Left Power", "%.2f", leftPower);
            telemetry.addData("Right Power", "%.2f", rightPower);
            telemetry.update();
        }
    }
}
