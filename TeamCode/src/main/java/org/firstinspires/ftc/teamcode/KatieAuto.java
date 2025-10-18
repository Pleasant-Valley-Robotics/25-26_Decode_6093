package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Katie's Auto", group="Gigi's")
public class KatieAuto extends AutoTemplate{
    final double HOLDTIME = 0.5;
    @Override
    public void runOpMode() {
        setupAuto();

        driveStraight(0.2, 10, 0);
        turnToHeading(0.2, -10);
        holdHeading(0.2, -10, HOLDTIME);

        driveStraight(0.2, 12.5, -10);
        turnToHeading(0.2, 0);
        holdHeading(0.2, 0, HOLDTIME);

        driveStraight(0.2, 33, 0);
        turnToHeading(0.2, -90);
        holdHeading(0.2, -90, HOLDTIME);

        driveStraight(0.2, 21, -90);
        turnToHeading(0.2, -180);
        holdHeading(0.2, -180, HOLDTIME);

        driveStraight(0.2, 21, -180);
        turnToHeading(0.2, -270);
        holdHeading(0.2, -270, HOLDTIME);

        driveStraight(0.2, 21, -270);
        turnToHeading(0.2, 0);
        holdHeading(0.2, 0, HOLDTIME);

        driveStraight(0.2, 21, 0);
        turnToHeading(0.2, -90);
        holdHeading(0.2, -90, HOLDTIME);

        driveStraight(0.2, 21, -90);
        turnToHeading(0.2, -180);
        holdHeading(0.2, -180, HOLDTIME);

        driveStraight(0.2, 41, -180);


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.

    }

}
