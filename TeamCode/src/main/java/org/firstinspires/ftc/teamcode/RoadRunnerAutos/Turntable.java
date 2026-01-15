package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Turntable {

    private Servo indexServo = null;
    private final double [] positions = {0.08, 0.25, 0.42, 0.59, 0.75, 0.92};

    private int positionId = 0;

    public Turntable(HardwareMap hardwareMap) {
        indexServo = hardwareMap.get(Servo.class, "index");
        updatePosition();
    }

    public void turnOnce() {
        positionId++;
        updatePosition();
    }

    private void turnToPosition(int id) {
        positionId = id;
        updatePosition();
    }


    private void updatePosition() {
        while (positionId < 0) positionId += 6;
        while (positionId > 5) positionId -= 6;


        indexServo.setPosition(positions[positionId]);
    }



}
