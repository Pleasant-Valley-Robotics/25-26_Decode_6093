package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import androidx.annotation.NonNull;

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

public class Shooter {

    private DcMotorEx shooter;
    private DcMotorEx shooter2;
    private Servo flickerServo;
    final public double upPos = 0.3;
    final public double downPos = .4;

    private boolean servoIsUp = false;
    private boolean inProcess = false;

    private int lastSpeed = 0;

    public Shooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        flickerServo = hardwareMap.get(Servo.class, "flicker");

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(BRAKE);
        shooter2.setZeroPowerBehavior(BRAKE);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1000, 0, 5, 10));
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1000, 0, 5, 10));
    }

    public void setServoPos(double position) {
        flickerServo.setPosition(position);
    }

    public Action shootOnce(Turntable turntable) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                flickerServo.setPosition(upPos);
                Actions.runBlocking(new SleepAction(0.2));
                flickerServo.setPosition(downPos);
                Actions.runBlocking(new SleepAction(0.2));
                turntable.turnLeft();
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
                Actions.runBlocking(new SleepAction(0.5));
                Actions.runBlocking(shootOnce(turntable));
                Actions.runBlocking(new SleepAction(0.5));
                Actions.runBlocking(shootOnce(turntable));
                Actions.runBlocking(new SleepAction(0.5));
                Actions.runBlocking(shootOnce(turntable));
                Actions.runBlocking(new SleepAction(0.5));
                return false;
            }
        };
    }

    public Action shootInPattern(Turntable turntable) {

        Turntable.IndexColors[] pattern = {Turntable.IndexColors.GREEN, Turntable.IndexColors.PURPLE, Turntable.IndexColors.PURPLE};

        if (turntable.countGreen() != 1 || turntable.getNumBalls() != 3) {
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
                Actions.runBlocking(new SleepAction(0.5));
                Actions.runBlocking(shootOnce(turntable));

                while (turntable.getBallAt(1) != pattern[index]) {
                    turntable.turnLeft();
                }
                index++;
                index %= 3;
                Actions.runBlocking(new SleepAction(0.5));
                Actions.runBlocking(shootOnce(turntable));

                while (turntable.getBallAt(1) != pattern[index]) {
                    turntable.turnLeft();
                }
                Actions.runBlocking(new SleepAction(0.5));
                Actions.runBlocking(shootOnce(turntable));

                return false;
            }
        };

    }



    public void spinUp(int speed) {
        lastSpeed = speed;
        shooter.setVelocity(speed);
        shooter2.setVelocity(speed);
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
