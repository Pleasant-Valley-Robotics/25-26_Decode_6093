package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Autonomous(name="Katie's Auto", group="Gigi's")  // All of these auto's were written and tested on the robot with the 312 motors, meaning this one will likely be
public class KatieAuto extends AutoTemplate{ // slightly off on it's intended robot
    final double HOLDTIME = 0.75;
    @Override
    public void runOpMode() {
        setupAuto();

        driveStraight(0.35, 65, 0);
        turnToHeading(0.2, -90);
        holdHeading(0.2, -90, HOLDTIME);

        driveStraight(0.35, 25, -90);
        turnToHeading(0.2, -180);
        holdHeading(0.2, -180, HOLDTIME);

        driveStraight(0.35, 21, -180);
        turnToHeading(0.2, -270);
        holdHeading(0.2, -270, HOLDTIME);

        driveStraight(0.35, 24, -270);
        turnToHeading(0.2, 0);
        holdHeading(0.2, 0, HOLDTIME);

        driveStraight(0.35, 24, 0);
        turnToHeading(0.2, -90);
        holdHeading(0.2, -90, HOLDTIME);

        driveStraight(0.35, 24, -90);
        turnToHeading(0.2, -180);
        holdHeading(0.2, -180, HOLDTIME);

        driveStraight(0.35, 35, -180);


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.

    }

}
