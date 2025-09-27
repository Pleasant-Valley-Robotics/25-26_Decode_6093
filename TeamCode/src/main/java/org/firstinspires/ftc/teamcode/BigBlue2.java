package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Autonomous(name="Blue Big Leave ver 2", group="Robot")
public class BigBlue2 extends AutoTemplate {
    @Override
    public void runOpMode() {
        setupAuto(); // Sets up things like the hardware map and motor modes (edit AutoTemplate when you change robot components)

        driveStraight(DRIVE_SPEED, 72.0, 0.0);    // Drive Forward 36"
        turnToHeading(TURN_SPEED, -155.0);        // Turn  CW to 90 Degrees
        holdHeading(TURN_SPEED, -155.0, 1.0);   // Hold 90 Deg heading for a 1/2 second

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message
    }

}
