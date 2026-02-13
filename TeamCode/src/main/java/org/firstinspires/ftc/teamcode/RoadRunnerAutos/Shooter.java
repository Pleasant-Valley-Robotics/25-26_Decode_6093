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
    final public double upPos = 0.3;
    final public double downPos = .4;

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
            if (turntable.getNumBalls() == 0) {
                return false;
            }

            if (!initialized) {
                    while (turntable.getBallAt(1) == null) {
                        turntable.turnLeft();
                        count++;

                        if (count > 300) {
                            throw new RuntimeException("infinite loop, yell at the programers");
                        }
                    }

                    initialized = true;
                    timer.reset();
                }

                if (timer.seconds() < 1.5 && timer.seconds() > 0.75) {
                    flickerServo.setPosition(upPos);
                } else if (timer.seconds() > 1.5) {
                    turntable.removeBall(1);
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
            int count = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isMoving()) return false;
                if (count >= 3) return false;

                if (timer.seconds() < .75 && timer.seconds() > 0.35) {
                    flickerServo.setPosition(upPos);
                } else if (timer.seconds() > .75 ){
                    flickerServo.setPosition(downPos);
                    turntable.removeBall(1);
                    turntable.turnLeft();
                    count++;
                    timer.reset();
                }
                return true;
            }
        };
    }


    public Action shootAllFAST(Turntable turntable) {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isMoving()) return false;
                if (turntable.getNumBalls() == 0) return false;

                if (timer.seconds() < .90 && timer.seconds() > .5) {
                    flickerServo.setPosition(upPos);
                } else if (timer.seconds() > 1.2){
                    flickerServo.setPosition(downPos);
                    turntable.removeBall(1);
                    turntable.turnLeft();
                    timer.reset();
                }
                return true;
            }
        };
    }

    public Action shootInPattern(Turntable turntable) {

        Turntable.IndexColors[] pattern = {Turntable.IndexColors.GREEN, Turntable.IndexColors.PURPLE, Turntable.IndexColors.PURPLE};

        if (turntable.countGreen() != 1 || turntable.getNumBalls() != 3) {
            return shootAll(turntable);
        }

        return new Action() {

            ElapsedTime timer = new ElapsedTime();
            int index = PoseStorage.shotsToCycle;
            int count = 0;
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isMoving()) return false;
                if (turntable.getNumBalls() == 0) return false;

                if (!initialized) {
                    timer.reset();
                    initialized = true;
                }

                while (turntable.getBallAt(1) != pattern[index]) {
                    turntable.turnLeft();
                    count++;

                    if (count > 300) {
                        throw new RuntimeException("infinite loop, yell at the programers");
                    }
                }
                if (timer.seconds() < .90 && timer.seconds() > .5) {
                    flickerServo.setPosition(upPos);
                } else if (timer.seconds() > 1.2){
                    flickerServo.setPosition(downPos);
                    turntable.removeBall(1);
                    index++;
                    index %= 3;
                    timer.reset();
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
    public int getLastSpeed() {return lastSpeed;}
    public boolean isMoving() {return shooter.getVelocity() > 100;}
    public boolean isAtSpeed() {
        return Math.abs(shooter.getVelocity() - lastSpeed) < 20;
    }

    public double getVelocity() {
        return shooter.getVelocity();
    }

}
