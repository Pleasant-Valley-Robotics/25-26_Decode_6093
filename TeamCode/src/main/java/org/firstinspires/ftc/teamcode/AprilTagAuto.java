package org.firstinspires.ftc.teamcode;
// It uses the logitech camera to identify the id of the april tags and then it decides to left or right (we just did this because to see if the code works)
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

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
        initAprilTag();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double currentHeading = getHeading();
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                if (!currentDetections.isEmpty()) {
                    AprilTagDetection targetAprilTag = currentDetections.get(0);
                    int id = targetAprilTag.id;
                    double targetHeading = currentHeading;
                    if (id == 21) { // Turns left
                        targetHeading = currentHeading - 90;
                    } else if (id == 22) {
                        targetHeading = currentHeading + 90;
                    }

                    turnToHeading(1, targetHeading);

                    telemetry.addLine(String.format("ID Detected: %d", id));
                    telemetry.addLine(String.format("Current heading: %.2f, Target heading: %.2f", currentHeading, targetHeading));
                    telemetry.update();
                    sleep(1000);

                } else {
                    telemetry.addLine("No AprilTag found.");
                    sleep(20);
                }

            }

            telemetry.addLine("Auto Complete.");
            telemetry.update();
            sleep(1000);  // Pause to display last telemetry message

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

}
