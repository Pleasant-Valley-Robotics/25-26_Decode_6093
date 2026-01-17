package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {

    private DcMotorEx shooter;
    private Servo flickerServo;
    final private double upPos = 0.365;
    final private double downPos = 0.45;

    private boolean servoIsUp = false;
    private boolean inProcess = false;


    private int lastSpeed = 0;
    public Shooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        flickerServo = hardwareMap.get(Servo.class, "flicker");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setZeroPowerBehavior(BRAKE);

        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
    }

    public Action fireOnce(Turntable turntable) {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (turntable.getNumBalls() == 0) {
                    return false;
                }

                while (turntable.getBallAt(turntable.getPositionId()) == null) {
                    turntable.turnLeft();
                }

                if (timer.seconds() < 0.2) {
                    flickerServo.setPosition(upPos);
                    turntable.removeBall(turntable.getNumBalls());
                } else {
                    turntable.turnLeft();
                    flickerServo.setPosition(downPos);
                    return false;
                }

                return true;
            }
        };
    }

    public Action shootInPattern(Turntable turntable) {
        return new Action() {
            private ElapsedTime feederTimer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isMoving()) return false;
                switch (turntable.getNumBalls()) {
                    case 3:
                        turntable.turnToPosition(turntable.getPositionId() + turntable.findIndexOf(Turntable.IndexColors.GREEN) - 4);
                        for (int i = 0; i < PoseStorage.shotsToCycle; i++) {
                            turntable.turnLeft();
                            turntable.turnLeft();
                        }
                        flickerServo.setPosition(upPos);
                        break;
                    case 2:
                    case 1:
                        turntable.turnLeft();
                        turntable.turnLeft();
                        break;
                    default:
                        turntable.turnLeft();
                        flickerServo.setPosition(downPos);
                        return false;
                }
                return true;
            }
        };
    }

    public Action shootAll(Turntable turntable) {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isMoving()) return false;
                if (!initialized) {
                    if (turntable.getPositionId() % 2 != 0) {
                        turntable.turnLeft();
                    }
                    initialized = true;
                }

                switch (turntable.getNumBalls()) {
                    case 3:
                    case 2:
                    case 1:
                        if (timer.seconds() < 0.5 && timer.seconds() > 0.2) {
                            flickerServo.setPosition(upPos);
                        } else if (timer.seconds() > 0.5 ){
                            flickerServo.setPosition(downPos);
                            turntable.removeBall(turntable.getPositionId());
                            turntable.turnLeft();
                            turntable.turnLeft();
                            timer.reset();
                        }

                        break;
                    default:
                        if (timer.seconds() > 0.2) {
                            turntable.turnLeft();
                            return false;
                        }
                }
                return true;
            }
        };
    }


    public void spinUp(int speed) {
        lastSpeed = speed;
        shooter.setVelocity(speed);
    }

    public void stop() {
        spinUp(0);
    }

    public boolean isMoving() {
        return shooter.getVelocity() > 100;
    }

    public boolean isAtSpeed() {
        return Math.abs(shooter.getVelocity() - lastSpeed) < 40;
    }

    public double getVelocity() {
        return shooter.getVelocity();
    }

}
