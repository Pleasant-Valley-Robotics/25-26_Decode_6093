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


    public void setPower(double power) {
        intake.setPower(power);
    }

    public Action autoIntake(Camera camera, Turntable turntable) {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            boolean isFirstTime = false;
            int count = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (turntable.getNumBalls() >= 3) {
                    return false;
                }
                if (camera.ballDetected()) {
                    if (!isFirstTime) {
                        while (turntable.getPositionId() % 2 == 0 || turntable.getBallAt(turntable.getPositionId()) != null) {
                            turntable.turnLeft();
                            count++;

                            if (count > 400) {
                                throw new RuntimeException("im gonna lose my mind");
                            }
                        }


                        timer.reset();
                        isFirstTime = true;
                    }

                    if (timer.seconds() > 0.4) {
                        turntable.addBall(turntable.getPositionId(), camera.getBallColor());
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
