/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels", and two servos
 * which feed that launcher.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */
@Disabled
@TeleOp(name = "EthanTeleop", group = "StarterBot")
public class EthanTeleop extends OpMode{

    private AprilTagProcessor aprilTag;
    private HuskyLens huskyLens;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested. (originally 0.20)
    final double TRIPLE_FEED_TIME_SECONDS = 2.50;
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */

    final double LAUNCHER_FAR_TARGET_VELOCITY = 1630;
    final double LAUNCHER_FAR_MIN_VELOCITY = 1630;
    final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1500; // Originally 1125
    final double LAUNCHER_CLOSE_MIN_VELOCITY = 1500;

    final double LAUNCHER_CYCLE_MIN_VELOCITY = 470;
    final double LAUNCHER_CYCLE_TARGET_VELOCITY = 480;

    final double LAUNCHER_REVERSE_MIN_VELOCITY = -500;
    final double LAUNCHER_REVERSE_TARGET_VELOCITY = -550;

    double LAUNCHER_ACTIVE_MIN_VELOCITY = LAUNCHER_CLOSE_MIN_VELOCITY;



    // Declare OpMode members.
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;


    final double FEED_TIME = .35;
    final double BACK_TIME = .2;
    ElapsedTime feederTimer = new ElapsedTime(20);
    ElapsedTime tripleFeederTime = new ElapsedTime();

    int targetAprilTag = 0;

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState leftLaunchState;
    private LaunchState rightLaunchState;

    // Setup a variable for each drive wheel to save power level for telemetry
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;

    boolean manualControl = true;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        initAprilTag();

        leftLaunchState = LaunchState.IDLE;
        rightLaunchState = LaunchState.IDLE;

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setDirection(DcMotor.Direction.REVERSE);
        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        frontLeftDrive.setZeroPowerBehavior(BRAKE);
        frontRightDrive.setZeroPowerBehavior(BRAKE);
        backLeftDrive.setZeroPowerBehavior(BRAKE);
        backRightDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFeeder.setDirection(DcMotorSimple.Direction.FORWARD);

        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        /*
         * Tell the driver that initialization is complete.
         */

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double rotate;


        double aprilTagTurnPower = getAprilTagTurnPower();

        if (gamepad1.xWasPressed()) {
            manualControl = !manualControl;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            if (!currentDetections.isEmpty()) {
                for (int i = 0; i < currentDetections.size(); i++ ) {
                    if (currentDetections.get(i).id == 24 || currentDetections.get(i).id == 20) {
                        targetAprilTag = currentDetections.get(i).id;
                    }
                }
            }
        }


        if (!manualControl) {
            rotate = aprilTagTurnPower;
        } else {
            rotate = gamepad1.right_stick_x;
        }

        arcadeDrive(forward, rotate);
        controlServos();

        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot.
         */
        if (gamepad2.x) {
            LAUNCHER_ACTIVE_MIN_VELOCITY = LAUNCHER_CLOSE_MIN_VELOCITY;
            launcher.setVelocity(LAUNCHER_CLOSE_TARGET_VELOCITY);
        } else if (gamepad2.a) { // stop flywheel
            launcher.setVelocity(STOP_SPEED);
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
        }
        else if (gamepad2.b){
            LAUNCHER_ACTIVE_MIN_VELOCITY = LAUNCHER_CYCLE_MIN_VELOCITY;
            launcher.setVelocity(LAUNCHER_CYCLE_TARGET_VELOCITY);
        } else if (gamepad2.y) {
            LAUNCHER_ACTIVE_MIN_VELOCITY = LAUNCHER_FAR_MIN_VELOCITY;
            launcher.setVelocity(LAUNCHER_FAR_TARGET_VELOCITY);
        } else if (gamepad2.left_trigger>0) {
            LAUNCHER_ACTIVE_MIN_VELOCITY = LAUNCHER_REVERSE_MIN_VELOCITY;
            launcher.setVelocity(LAUNCHER_REVERSE_TARGET_VELOCITY);
            leftFeeder.setPower(-1);
            rightFeeder.setPower(-1);
        }
        /*
         * Now we call our "Launch" function.
         */
        if (rightLaunchState == LaunchState.IDLE) {
            //launch(gamepad1.leftBumperWasPressed());
        }
        if (leftLaunchState == LaunchState.IDLE) {
            //tripleLaunch(gamepad1.rightBumperWasPressed());
        }
        if(gamepad2.right_bumper){
            feederTimer.reset();
        }
        /*
         * Show the state and motor powers
         */
        telemetry.addData("State", leftLaunchState);
        telemetry.addData("motorSpeed", launcher.getVelocity());
        telemetry.addData("Servo Direction R", rightFeeder.getDirection());
        telemetry.addData("Servo Direction L", leftFeeder.getDirection());
        telemetry.addData("Servo Power R", rightFeeder.getPower());
        telemetry.addData("Servo Power L", rightFeeder.getPower());


        telemetry.addData("AprilTag Turn Power", aprilTagTurnPower);

        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    void arcadeDrive(double forward, double rotate) {
        double y = forward;
        double x = gamepad1.left_stick_x * 1.1; // Strafe
        double r = rotate;

        double frontLeftPower  = y + x + r;
        double frontRightPower = y - x - r;
        double backLeftPower   = y - x + r;
        double backRightPower  = y + x - r;

        double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }

    void tripleLaunch(boolean shotRequested) {
        switch (rightLaunchState) {
            case IDLE:
                if (shotRequested) {
                    rightLaunchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_CLOSE_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_CLOSE_MIN_VELOCITY) {
                    rightLaunchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                tripleFeederTime.reset();
                rightLaunchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (tripleFeederTime.seconds() > TRIPLE_FEED_TIME_SECONDS) {
                    rightLaunchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }


    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }


        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }

    private void controlServos() {
        if (feederTimer.seconds() < BACK_TIME) {
            rightFeeder.setPower(-1);
            leftFeeder.setPower(-1);
        } else if (feederTimer.seconds() < BACK_TIME + FEED_TIME) {
            rightFeeder.setPower(1);
            leftFeeder.setPower(1);
        } else {
            rightFeeder.setPower(0);
            leftFeeder.setPower(0);
        }
    }


    /**
     * Calculates the turn power needed to align with AprilTag ID 24.
     * @return The calculated turn power, or 0.0 if the tag is not visible.
     */
    private double getAprilTagTurnPower() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == targetAprilTag) {
                double tolerance = 0.75; // Tolerance in inches
                double deviation = -detection.ftcPose.z;


                if (Math.abs(deviation) > tolerance) {
                    double kP = 0.02;
                    double turnPower = kP * deviation;


                    return Math.max(-0.4, Math.min(0.4, turnPower));
                } else {
                    // We are aligned, so command no turn.
                    return 0.0;
                }
            }
        }
        return 0.0;
    }

    private double getAprilTagShotPower() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == targetAprilTag) {
                double distance = -detection.ftcPose.x;

                return (distance * 12.5);
            }
        }
        return 0.0;
    }


}
