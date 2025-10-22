package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="Abby Auto", group="Gigi's")  // All of these auto's were written and tested on the robot with the 312 motors, meaning this one will likely
public class AbbyAuto extends AutoTemplate{ // be slightly off on it's intended robot
    final double HOLDTIME = 0.75;
    @Override
    public void runOpMode() {
        setupAuto();
        // To first cone
        driveStraight(0.2, 42.5, 0);
        turnToHeading(0.1, 90);
        holdHeading(0.1, 90, HOLDTIME);

        // First cone
        driveStraight(0.2, 17, 90);
        turnToHeading(0.1, 0);
        holdHeading(0.1, 0, HOLDTIME);

        driveStraight(0.2, 19, 0);
        turnToHeading(0.1, 270);
        holdHeading(0.1, 270, HOLDTIME);

        driveStraight(0.2, 17, 270);
        turnToHeading(0.1, 180);
        holdHeading(0.1, 180, HOLDTIME);

        driveStraight(0.2, 19, 180);
        turnToHeading(0.1, 270);
        holdHeading(0.1, 270, HOLDTIME);

        // Loop the loop (these values probably aren't correct yet)
        driveStraight(0.2, 26, 270);
        turnToHeading(0.1, 0);
        holdHeading(0.1, 0, HOLDTIME);

        driveStraight(0.2, 24, 0);
        turnToHeading(0.1, 90);
        holdHeading(0.1, 90, HOLDTIME);

        driveStraight(0.2, 24, 90);
        turnToHeading(0.1, 180);
        holdHeading(0.1, 180, HOLDTIME);

        driveStraight(0.2, 24, 180);
        turnToHeading(0.1, 270);
        holdHeading(0.1, 270, HOLDTIME);

        driveStraight(0.2, 43, 270);
        turnToHeading(0.1, 0);
        holdHeading(0.1, 0, HOLDTIME);

        driveStraight(0.2, 19, 0);
        turnToHeading(0.1, 90);
        holdHeading(0.1, 90, HOLDTIME);

        driveStraight(0.2, 16, 90);
        turnToHeading(0.1, 180);
        holdHeading(0.1, 180, HOLDTIME);

        driveStraight(0.2, 30,  180);


        // Blue cone





        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.

    }

}
