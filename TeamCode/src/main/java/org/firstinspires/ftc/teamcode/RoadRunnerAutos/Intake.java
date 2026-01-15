package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intake;
    private boolean runIntake = false;
    private ColorSensor colorSensor;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        colorSensor = hardwareMap.get(ColorSensor.class, "location4");
    }


    public Action powerIntake() {
        runIntake = true;
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (runIntake) {
                    intake.setPower(1);
                } else {
                    return false;
                }

                return true;
            }
        };
    }

    public Action autoIndex(Turntable turntable, int motifId) {

        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return false;
            }
        };
    }



}
