package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="Abby Auto", group="Gigi's") // This is finished for the robot with the 435 motors.
public class AbbyAuto extends AutoTemplate{
    final double HOLDTIME = 1.65;
    @Override
    public void runOpMode() {
        setupAuto();
        // To first cone
        driveStraight(0.1, 48.5, 0);
        holdHeading(0.1, 0, 0.3); // Added as a wait because it originally wasn't stopping fully

        turnToHeading(0.1, 90);
        holdHeading(0.1, 90, HOLDTIME);

        // First cone
        driveStraight(0.1, 17, 90);
        turnToHeading(0.1, 0);
        holdHeading(0.1, 0, HOLDTIME);

        driveStraight(0.1, 25, 0);
        turnToHeading(0.1, 270);
        holdHeading(0.1, 270, HOLDTIME);

        driveStraight(0.1, 22.5, 270);
        turnToHeading(0.1, 180);
        holdHeading(0.1, 180, HOLDTIME);

        driveStraight(0.1, 25, 180);
        turnToHeading(0.1, 270);
        holdHeading(0.1, 270, HOLDTIME);

        // Loop the loop
        driveStraight(0.1, 30, 270);
        turnToHeading(0.1, 0);
        holdHeading(0.1, 0, HOLDTIME);

        driveStraight(0.1, 27.5, 0);
        turnToHeading(0.1, 90);
        holdHeading(0.1, 90, HOLDTIME);

        driveStraight(0.1, 24, 90);
        turnToHeading(0.1, 180);
        holdHeading(0.1, 180, HOLDTIME);

        driveStraight(0.1, 27.5, 180);
        turnToHeading(0.1, 270);
        holdHeading(0.1, 270, HOLDTIME);

        driveStraight(0.1, 47.5, 270);
        turnToHeading(0.1, 0);
        holdHeading(0.1, 0, HOLDTIME);

        driveStraight(0.1, 21.5, 0);
        turnToHeading(0.1, 90);
        holdHeading(0.1, 90, HOLDTIME);

        driveStraight(0.1, 18.5, 90);
        turnToHeading(0.1, 180);
        holdHeading(0.1, 180, HOLDTIME);

        driveStraight(0.1, 35,  180);


        // Blue cone





        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.

    }

}
