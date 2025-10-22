package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name="Gavin's Auto", group="Gigi's") // All of these auto's were written and tested on the robot with the 312 motors, meaning this one will be the only
public class GavinAuto extends AutoTemplate{ // one that will work as intended
    final double HOLDTIME = 0.75;

    @Override
    public void runOpMode() {
        setupAuto();

        driveStraight(0.2, 15, 0);
        turnToHeading(0.1, 90);
        holdHeading(0.1, 90, HOLDTIME);

        driveStraight(0.2, 15, 90);
        turnToHeading(0.1, 0);
        holdHeading(0.1, 0, HOLDTIME);

        driveStraight(0.2, 13.5, 0);
        turnToHeading(0.1, 270);
        holdHeading(0.1, 270, HOLDTIME);

        driveStraight(0.2, 17, 270);
        turnToHeading(0.1, 0);
        holdHeading(0.1, 0, HOLDTIME);

        driveStraight(0.2, 13.5, 0);
        turnToHeading(0.1, 90);
        holdHeading(0.1, 90, HOLDTIME);

        driveStraight(0.2, 19, 90);
        turnToHeading(0.1, 0);
        holdHeading(0.1, 0, HOLDTIME);

        driveStraight(0.2, 25, 0);
        turnToHeading(0.1, 270);
        holdHeading(0.1, 270, HOLDTIME);

        driveStraight(0.2, 62.5, 270);
        turnToHeading(0.1, 180);
        holdHeading(0.1, 180, HOLDTIME);

        driveStraight(0.2, 42.5, 180);
        turnToHeading(0.1, 90);
        holdHeading(0.1, 90, HOLDTIME);

        driveStraight(0.2, 4, 90);





        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.

    }

}
