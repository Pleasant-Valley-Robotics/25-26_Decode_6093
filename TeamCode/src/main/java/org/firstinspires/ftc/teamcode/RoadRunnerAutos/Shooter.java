package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
public class Shooter {

    private DcMotorEx shooter;
    private DcMotorEx shooter2;
    private Servo flickerServo;
    private Servo led;
    final public double upPos = 0.38;
    final public double downPos = .43;
    public static double p = 500;
    public static double d = 0;
    public static double f = 12.5;

    private boolean servoIsUp = false;
    private boolean inProcess = false;

    private int lastSpeed = 0;

    public Shooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        flickerServo = hardwareMap.get(Servo.class, "flicker");
        led = hardwareMap.get(Servo.class, "led");

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(BRAKE);
        shooter2.setZeroPowerBehavior(BRAKE);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, 0, d, f));
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, 0, d, f));
    }

    public void setServoPos(double position) {
        flickerServo.setPosition(position);
    }

    public Action shootOnceL(Turntable turntable) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                flickerServo.setPosition(upPos);
                Actions.runBlocking(new SleepAction(0.2));
                flickerServo.setPosition(downPos);
                Actions.runBlocking(new SleepAction(0.2));
                turntable.removeBall(1);
                turntable.turnLeft();
                return false;
            }
        };
    }

    public Action shootOnceR(Turntable turntable) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                flickerServo.setPosition(upPos);
                Actions.runBlocking(new SleepAction(0.10));
                flickerServo.setPosition(downPos);
                Actions.runBlocking(new SleepAction(0.10));
                turntable.removeBall(1);
                turntable.turnRight();
                return false;
            }
        };
    }

    public Action shootAll(Turntable turntable) {

        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isMoving()) return false;
                turntable.turnToPosition(0);
                Actions.runBlocking(new SleepAction(0.2));
                Actions.runBlocking(shootOnceL(turntable));
                Actions.runBlocking(new SleepAction(0.2));
                Actions.runBlocking(shootOnceL(turntable));
                Actions.runBlocking(new SleepAction(0.2));
                Actions.runBlocking(shootOnceL(turntable));
                Actions.runBlocking(new SleepAction(0.2));
                return false;
            }
        };


    }

    private enum ShootFastStates {
        START,
        FLICKERUP,
        FLICKERDOWN,
    }

    public Action shootFastTele(Turntable turntable, double delay, double ratio) {
        return new Action() {
            ShootFastStates launchStates;
            boolean init = false;
            ElapsedTime timer = new ElapsedTime();
            int count = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double updatedDelay = (launchStates == ShootFastStates.FLICKERUP
                        ? delay : delay * ratio);

                if (!init) {
                    init = true;
                    launchStates = ShootFastStates.START;

                    if (!isMoving()) return false;
                    if (turntable.getPositionId() == 2) {
                        updatedDelay = 0;
                    }
                    turntable.turnToPosition(2);
                    timer.reset();
                }


                if (timer.seconds() > updatedDelay * 2) {
                    if (launchStates == ShootFastStates.START) {
                        launchStates = ShootFastStates.FLICKERUP;
                        flickerServo.setPosition(upPos);
                        timer.reset();
                    } else if (launchStates == ShootFastStates.FLICKERUP) {
                        launchStates = ShootFastStates.FLICKERDOWN;
                        flickerServo.setPosition(downPos);
                        turntable.removeBall(1);
                        turntable.turnRight();
                        timer.reset();
                    } else if (launchStates == ShootFastStates.FLICKERDOWN) {
                        if (count == 2) {
                            launchStates = ShootFastStates.START;
                            return false;
                        } else {
                            launchStates = ShootFastStates.FLICKERUP;
                            flickerServo.setPosition(upPos);
                            count++;
                            timer.reset();
                        }
                    }
                }
                return true;
            }
        };
    }

    public Action shootFast(Turntable turntable, double delay) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isMoving()) return false;
                turntable.turnToPosition(0);
                Actions.runBlocking(new SleepAction(delay));
                Actions.runBlocking(shootOnceL(turntable));
                Actions.runBlocking(new SleepAction(delay));
                Actions.runBlocking(shootOnceL(turntable));
                Actions.runBlocking(new SleepAction(delay));
                Actions.runBlocking(shootOnceL(turntable));
                Actions.runBlocking(new SleepAction(delay));
                return false;
            }
        };
    }


    public Action shootInPattern(Turntable turntable, double delay) {

        Turntable.IndexColors[] pattern = {Turntable.IndexColors.GREEN, Turntable.IndexColors.PURPLE, Turntable.IndexColors.PURPLE};

        if (turntable.countGreen() != 1 || turntable.getNumBalls() != 3 || PoseStorage.shotsToCycle == -1) {
            return shootAll(turntable);
        }

        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                int index = PoseStorage.shotsToCycle;

                while (turntable.getBallAt(1) != pattern[index]) {
                    turntable.turnLeft();
                }
                index++;
                index %= 3;
                Actions.runBlocking(new SleepAction(delay));
                Actions.runBlocking(shootOnceL(turntable));

                while (turntable.getBallAt(1) != pattern[index]) {
                    turntable.turnLeft();
                }
                index++;
                index %= 3;
                Actions.runBlocking(new SleepAction(delay));
                Actions.runBlocking(shootOnceL(turntable));

                while (turntable.getBallAt(1) != pattern[index]) {
                    turntable.turnLeft();
                }
                Actions.runBlocking(new SleepAction(delay));
                Actions.runBlocking(shootOnceL(turntable));

                return false;
            }
        };

    }



    public void spinUp(int speed) {
        lastSpeed = speed;
        shooter.setVelocity(speed);
        shooter2.setVelocity(speed);
    }

    public void setLedIntensity(double intensity) {
        led.setPosition(intensity);
    }

    public void stop() {
        spinUp(0);
    }
    public int getLastSpeed() {return lastSpeed;}
    public boolean isMoving() {return shooter.getVelocity() > 100;}
    public boolean isAtSpeed() {
        return Math.abs(shooter.getVelocity() - lastSpeed) < 45;
    }

    public double getVelocity() {
        return shooter.getVelocity();
    }

    public void updatePID() {
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, 0, d, f));
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, 0, d, f));
    }

}
