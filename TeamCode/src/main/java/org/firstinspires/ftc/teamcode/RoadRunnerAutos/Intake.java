package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    private DcMotor intake;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void setPower(double power) {intake.setPower(power);}

    public Action normalIntake(Camera camera, Turntable turntable) {
        return new Action() {
            ElapsedTime timer = new ElapsedTime(0);

            boolean isFirstTime = false;
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (turntable.getNumBalls() >= 3) {
                    turntable.turnLeft();
                    return false;
                }

                if (!initialized) {
                    turntable.turnToPosition(0);
                    initialized = true;
                }

                if (camera.ballDetected() && timer.seconds() > 0.2) {
                    if (!isFirstTime && timer.seconds() > 0.40) {
                        timer.reset();
                        isFirstTime = true;
                    }

                    if (timer.seconds() > 0.65) {
                        turntable.addBall(0, camera.getBallColor());
                        turntable.turnLeft();
                        timer.reset();
                        isFirstTime = false;
                    }
                }
                return true;
            }
        };
    }


    public Action autoIntake(Camera camera, Turntable turntable) {
        return new Action() {
            ElapsedTime timer = new ElapsedTime(5);
            ElapsedTime limit = new ElapsedTime(0);

            boolean isFirstTime = false;
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    limit.reset();
                    turntable.turnToPosition(0);
                    initialized = true;
                }

                if (turntable.getNumBalls() >= 3 || limit.seconds() > 5) {
                    turntable.turnLeft();
                    return false;
                }

                if (camera.ballDetected()) {
                    if (!isFirstTime && timer.seconds() > 0.2) {
                        timer.reset();
                        isFirstTime = true;

                    }

                    if (timer.seconds() > 0.60) {
                        turntable.addBall(0, camera.getBallColor());
                        turntable.turnLeft();
                        timer.reset();
                        isFirstTime = false;
                    }
                }
                return true;
            }
        };
    }


    public void stopIntake() {
        intake.setPower(0);
    }


}
