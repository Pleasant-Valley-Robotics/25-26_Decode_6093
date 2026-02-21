package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunnerAutos.Camera;

@TeleOp(name="Sensors", group="Test")
public class Sensors extends OpMode {
    private Camera camera;
    private DistanceSensor loc1;
    private DistanceSensor loc2;

    public void init() {
        camera = new Camera(hardwareMap);
        loc1 = hardwareMap.get(DistanceSensor.class, "location1");
        loc2 = hardwareMap.get(DistanceSensor.class, "location2");
    }

    public void loop() {
        telemetry.addData("Location 1 (mm)", loc1.getDistance(DistanceUnit.MM));
        telemetry.addData("Location 2 (mm)", loc2.getDistance(DistanceUnit.MM));
        telemetry.addData("Ball Detected R", camera.ballDetectedR());
        telemetry.addData("Ball Detected L", camera.ballDetectedL());

        telemetry.update();
    }


}
