package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Big Leave ver 2", group="Robot")
public class BigBlue2 extends AutoTemplate {
    @Override
    public void runOpMode() {
        setupAuto(); // Sets up things like the hardware map and motor modes (edit AutoTemplate when you change robot components)

        driveStraightWithOdo(DRIVE_SPEED * 0.5, 33.0, 0.0);

        driveStraightWithOdo(DRIVE_SPEED * 0.5, -33.0, 0.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message
    }
}
