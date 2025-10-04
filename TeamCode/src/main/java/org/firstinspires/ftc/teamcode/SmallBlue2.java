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
@Autonomous(name="Blue Small Leave", group="Robot")
public class SmallBlue2 extends AutoTemplate {
    //This auto scores 3 points by leaving the launch line and getting ready to shoot
    @Override
    public void runOpMode() {
        setupAuto(); // Sets up things like the hardware map and motor modes (edit AutoTemplate when you change robot components)

        driveStraight(DRIVE_SPEED, 96.0, 0.0);    // Drive Forward 96"
        turnToHeading(TURN_SPEED, 60.0);        // Turn  CW to 60 Degrees
        holdHeading(TURN_SPEED, 60.0, 1.0);   // Hold 60 Deg heading for a 1 second

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message
    }

}
