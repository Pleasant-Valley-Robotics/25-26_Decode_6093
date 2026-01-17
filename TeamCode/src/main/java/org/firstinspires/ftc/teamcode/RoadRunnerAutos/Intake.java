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
            boolean isFirstTime = true;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                int posToTurnTo;
                switch (turntable.getNumBalls()) {
                    case 0:
                        posToTurnTo = 1;
                        break;
                    case 1:
                        posToTurnTo = 3;
                        break;
                    case 2:
                        posToTurnTo = 5;
                        break;
                    default:
                        turntable.turnLeft();
                        return false;
                }

                turntable.turnToPosition(posToTurnTo);

                if (camera.ballDetected()) {
                    if (!isFirstTime) {
                        timer.reset();
                        isFirstTime = true;
                    }

                    if (timer.seconds() > 0.5) {
                        turntable.addBall(posToTurnTo, camera.getBallColor());
                        turntable.turnLeft();
                        turntable.turnLeft();
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
