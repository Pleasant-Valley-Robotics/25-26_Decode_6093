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
                setPower(-1);
                return false;
            }
        };
    }

    public Action normalIntake(Camera camera, Turntable turntable) {
        return new Action() {
            ElapsedTime timerR = new ElapsedTime(0);
            ElapsedTime timerL = new ElapsedTime(0);
            ElapsedTime checkBall = new ElapsedTime(0);
            char side = ' '; // space for unknown, r for right, l for left

            int count = 0;


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if ((side == 'l' || side == ' ') && camera.ballDetectedL()) {
                    side = 'l';
                    Actions.runBlocking(new SleepAction(0.2));
                    turntable.turnLeft();
                    count++;
                    Actions.runBlocking(new SleepAction(0.5));
                }

                if ((side == 'r' || side == ' ') && camera.ballDetectedR()) {
                    side = 'r';
                    Actions.runBlocking(new SleepAction(0.2));
                    turntable.turnRight();
                    count++;
                    Actions.runBlocking(new SleepAction(0.5));
                }


                return count < 3;
            }
        };
    }

    public Action autoIntake(Camera camera, Turntable turntable) {
        return new Action() {
            ElapsedTime limiter = new ElapsedTime(0);
            char side = ' '; // space for unknown, r for right, l for left

            int count = 0;


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if ((side == 'l' || side == ' ') && camera.ballDetectedL()) {
                    side = 'l';
                    Actions.runBlocking(new SleepAction(0.2));
                    turntable.turnLeft();
                    count++;
                    Actions.runBlocking(new SleepAction(0.5));
                }

                if ((side == 'r' || side == ' ') && camera.ballDetectedR()) {
                    side = 'r';
                    Actions.runBlocking(new SleepAction(0.2));
                    turntable.turnRight();
                    count++;
                    Actions.runBlocking(new SleepAction(0.5));
                }


                return count < 3 || limiter.seconds() > 6.7 ;
            }
        };
    }


    public void stopIntake() {
        intake.setPower(0);
    }


}
