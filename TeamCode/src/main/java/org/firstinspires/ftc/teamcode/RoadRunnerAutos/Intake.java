package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
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
    public Action reverse() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setPower(-.8);
                return false;
            }
        };
    }

    public Action normalIntake(Camera camera, Turntable turntable) {
        return new Action() {
            char side = ' '; // space for unknown, r for right, l for left

            int count = 0;


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if ((side == 'l' || side == ' ') && camera.ballDetectedL()) {
                    side = 'l';
                    Actions.runBlocking(new SleepAction(0.35));
                    turntable.turnLeft();
                    count++;
                    Actions.runBlocking(new SleepAction(0.65));
                }

                if ((side == 'r' || side == ' ') && camera.ballDetectedR()) {
                    side = 'r';
                    Actions.runBlocking(new SleepAction(0.35));
                    turntable.turnRight();
                    count++;
                    Actions.runBlocking(new SleepAction(0.65));
                }


                return count < 3;
            }
        };
    }

    public Action autoIntake(Camera camera, Turntable turntable, double limitTime) {
        return new Action() {
            ElapsedTime limiter = new ElapsedTime(0);
            char side = ' '; // space for unknown, r for right, l for left
            boolean init = false;
            int count = 0;


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    limiter.reset();
                    init = true;
                }


                if ((side == 'l' || side == ' ') && camera.ballDetectedL()) {
                    side = 'l';
                    Actions.runBlocking(new SleepAction(0.28));
                    turntable.turnLeft();
                    count++;
                    Actions.runBlocking(new SleepAction(0.57));
                }

                if ((side == 'r' || side == ' ') && camera.ballDetectedR()) {
                    side = 'r';
                    Actions.runBlocking(new SleepAction(0.28));
                    turntable.turnRight();
                    count++;
                    Actions.runBlocking(new SleepAction(0.57));
                }

                if (limiter.seconds() > limitTime) {
                    return false;
                }
                return count < 3;
            }
        };
    }

    public Action aut2oIntake(Camera camera, Turntable turntable, double limitTime) {
        return new Action() {
            ElapsedTime limiter = new ElapsedTime(0);
            char side = ' '; // space for unknown, r for right, l for left
            boolean init = false;
            int count = 0;
            boolean hasBallL = false;
            boolean hasBallR = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    limiter.reset();
                    init = true;
                }



                if ((side == ' ') && camera.ballDetectedL()) {
                    Actions.runBlocking(new SleepAction(0.5));
                    turntable.turnLeft();
                    Actions.runBlocking(new SleepAction(0.5));
                    side = 'l';
                    count++;
                } else if ((side == ' ') && camera.ballDetectedR()) {
                    Actions.runBlocking(new SleepAction(0.5));
                    turntable.turnRight();
                    Actions.runBlocking(new SleepAction(0.5));
                    side = 'r';
                    count++;
                } else if (camera.ballDetectedR() && !hasBallL) {
                    hasBallL = true;
                    count++;
                } else if (camera.ballDetectedL() && !hasBallR) {
                    hasBallR = true;
                    count++;
                }

                if (limiter.seconds() > limitTime) {
                    return false;
                }
                return count < 3;
            }
        };
    }


    public void stopIntake() {
        intake.setPower(0);
    }


}
