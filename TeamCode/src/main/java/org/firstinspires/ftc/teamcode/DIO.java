/* Copyright (c) 2021 FIRST. All rights reserved.
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
* Neither the name of FIRST nor the names of its contributors may be used to endorse or
* promote products derived from this software without specific prior written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
* LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="DIO", group="Linear OpMode")
public class DIO extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    private ColorSensor location1 = null;
    private ColorSensor location2 = null;
    private ColorSensor location3 = null;

    private DcMotor shooter = null;

    private DcMotor intake = null;

    private Servo indexServo = null;
    private Servo flickerServo = null;


    private int minPurpleValue = 90;
    private int minGreenValue = 260;
    private double[] intakes = {0.73, 0.40, 0.05};
    private double[] shoots = {0.90, 0.55, 0.22};
    private IndexColors[] indexState = new IndexColors[3];

    private int targetAprilTag = 22; // Two cycles
    private int currentIntake = 0;
    private double angle = intakes[2];

    public enum IndexColors {
        NONE,
        PURPLE,
        GREEN
    }

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        location1 = hardwareMap.get(ColorSensor.class, "location1");
        location2 = hardwareMap.get(ColorSensor.class, "location2");
        location3 = hardwareMap.get(ColorSensor.class, "location3");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        indexServo = hardwareMap.get(Servo.class, "index");
        flickerServo = hardwareMap.get(Servo.class, "flicker");


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            double intakePower = -gamepad2.right_stick_y;
            double shooterPower = -gamepad2.left_stick_y;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);
            shooter.setPower(shooterPower);
            intake.setPower(intakePower);


            if (gamepad2.leftBumperWasPressed()) angle = intakes[0];
            if (gamepad2.dpad_down) angle = intakes[1];
            if (gamepad2.dpad_right) angle = intakes[2];
            if (gamepad2.rightBumperWasPressed()) angle = shoots[0];
            if (gamepad2.dpad_up) angle = shoots[1];
            if (gamepad2.dpad_left) angle = shoots[2];
            if (gamepad2.xWasPressed()) fireAll();

            if (gamepad2.aWasPressed()) {
                flickerServo.setPosition(0.46);
            }
            if (gamepad2.bWasPressed()) {
                flickerServo.setPosition(0.37);
            }

            if (indexServo.getPosition() == shoots[0]) {
                indexState = updateIndexStates(indexState);
            }


            if (intakePower < 0) {
                if (currentIntake < intakes.length && ballDetected(currentIntake)) {
                    currentIntake++;
                    if (currentIntake >= intakes.length) {
                        angle = shoots[0];
                    } else {
                        angle = intakes[currentIntake];
                    }
                } else if (currentIntake < intakes.length) {
                    // Rattle between shoot and intake
                    if (Math.abs(indexServo.getPosition() - intakes[currentIntake]) < 0.01) {
                        angle = shoots[currentIntake];
                    } else if (Math.abs(indexServo.getPosition() - shoots[currentIntake]) < 0.01){
                        angle = intakes[currentIntake];
                    }
                }
            }



            indexServo.setPosition(angle);


            // Show the elapsed game time and wheel power.

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("index servo", indexServo.getPosition());
            telemetry.addData("flkicker servo", flickerServo.getPosition());
            telemetry.addData("Location 1 raw", location1.green());
            telemetry.addData("Location 2 raw", location2.green());
            telemetry.addData("Location 3 raw", location3.green());
            telemetry.addData("Location 1", indexState[0]);
            telemetry.addData("Location 2", indexState[1]);
            telemetry.addData("Location 3", indexState[2]);
            telemetry.addData("Servo Position", angle);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.update();
        }
    }

    private boolean ballDetected(int currentIntake) {
        if (currentIntake == 0) return location1.green() > minPurpleValue;
        if (currentIntake == 1) return location2.green() > minPurpleValue;
        return location3.green() > minPurpleValue;
    }
    private void fireAll() {
        for (double intakePos : intakes) {
            indexServo.setPosition(intakePos);
            while (Math.abs(indexServo.getPosition() - intakePos) > 0.01) {
                continue;
            }
            flickerServo.setPosition(0.37);
            while (Math.abs(flickerServo.getPosition() - 0.37) > 0.01) {
                continue;
            }
            flickerServo.setPosition(0.46);
            while (Math.abs(flickerServo.getPosition() - 0.46) > 0.01) {
                continue;
            }

        }
    }

    private IndexColors[] updateIndexStates(IndexColors[] state) {
        IndexColors[] newState = new IndexColors[3];
        if (location1.green() > minPurpleValue) {
            if (location1.green() > minGreenValue) {
                newState[0] = IndexColors.GREEN;
            } else {
                newState[0] = IndexColors.PURPLE;
            }
        } else {
            newState[0] = state[0];
        }

        if (location2.green() > minPurpleValue) {
            if (location2.green() > minGreenValue) {
                newState[1] = IndexColors.GREEN;
            } else {
                newState[1] = IndexColors.PURPLE;
            }
        } else {
            newState[1] = state[1];
        }

        if (location3.green() > minPurpleValue) {
            if (location3.green() > minGreenValue) {
                newState[2] = IndexColors.GREEN;
            } else {
                newState[2] = IndexColors.PURPLE;
            }
        } else {
            newState[2] = state[2];
        }
        return newState;
    }



}
