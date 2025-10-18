package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Gavin's Auto", group="Gigi's")
public class GavinAuto extends AutoTemplate{
    @Override
    public void runOpMode() {
        setupAuto();





        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.

    }

}
