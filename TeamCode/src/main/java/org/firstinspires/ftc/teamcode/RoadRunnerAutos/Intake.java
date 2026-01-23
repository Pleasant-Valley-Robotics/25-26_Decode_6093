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

    public Action autoIntake(Camera camera, Turntable turntable) {
        return new Action() {
            ElapsedTime timer = new ElapsedTime(5);
            ElapsedTime limit = new ElapsedTime();

            boolean isFirstTime = false;
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (turntable.getNumBalls() >= 3) {
                    return false;
                }

                if (!initialized) {
                    turntable.turnToPosition(1);
                    initialized = true;
                }

                if (camera.ballDetected()) {
                    if (!isFirstTime && timer.seconds() > 0.22) {
                        timer.reset();
                        isFirstTime = true;

                    }

                    if (timer.seconds() > 0.42) {
                        turntable.addBall(0, camera.getBallColor());
                        turntable.turnLeft();
                        turntable.turnLeft();
                        timer.reset();
                        isFirstTime = false;
                    }
                }
                return true;
            }
        };
    }


    public Action autoIntakeFAST(Camera camera, Turntable turntable) {
        return new Action() {
            ElapsedTime limit = new ElapsedTime();
            ElapsedTime timer = new ElapsedTime(5);
            boolean isFirstTime = false;
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (turntable.getNumBalls() >= 3 || limit.seconds() > 2) {
                    return false;
                }

                if (!initialized) {
                    turntable.turnToPosition(1);
                    initialized = true;
                }

                if (camera.ballDetected()) {
                    if (!isFirstTime && timer.seconds() > 0.25) {
                        timer.reset();
                        isFirstTime = true;

                    }

                    if (timer.seconds() > 0.4) {
                        turntable.addBall(0, camera.getBallColor());
                        if (turntable.getNumBalls() >= 3) {
                            return false;
                        }
                        turntable.turnLeft();
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
