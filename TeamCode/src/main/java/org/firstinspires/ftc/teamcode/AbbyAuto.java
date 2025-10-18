package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Abby Auto", group="Gigi's")
public class AbbyAuto extends AutoTemplate{
    @Override
    public void runOpMode() {
        setupAuto();





        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.

    }

}
