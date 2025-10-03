package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name="April Tag Auto", group="Robot")
public class AprilTagAuto extends AutoTemplate {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;


    @Override
    public void runOpMode() {
        setupAuto(); // Sets up things like the hardware map and motor modes (edit AutoTemplate when you change robot components)

        driveStraight(DRIVE_SPEED, 33.0, 0.0);    // Drive Forward 36"
        turnToHeading(TURN_SPEED, 155.0);        // Turn  CW to 90 Degrees
        holdHeading(TURN_SPEED, 155.0, 1.0);   // Hold 90 Deg heading for a 1/2 second

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message
    }

}
