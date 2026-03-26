package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import com.acmerobotics.roadrunner.Vector2d;

public class NewVector {
    public double x;
    public double y;

    public NewVector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2d getVector2d() {
        return new Vector2d(x, y * PoseStorage.isRed);
    }

}
