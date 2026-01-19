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
    final public double upPos = 0.365;
    final public double downPos = 0.45;

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

    public void setServoPos(double position) {
        flickerServo.setPosition(position);
    }

    public Action fireOnce(Turntable turntable) {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            boolean initialized = false;
            int count = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    if (turntable.getNumBalls() == 0) {
                        return false;
                    }

                    while (turntable.getBallAt(4) == null) {
                        turntable.turnLeft();
                        count++;

                        if (count > 40) {
                            throw new RuntimeException("Kill yourself");
                        }
                    }

                    initialized = true;
                    timer.reset();
                }

                if (timer.seconds() < 3 && timer.seconds() > 1) {
                    flickerServo.setPosition(upPos);
                } else if (timer.seconds() > 3) {
                    turntable.removeBall(4);
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
