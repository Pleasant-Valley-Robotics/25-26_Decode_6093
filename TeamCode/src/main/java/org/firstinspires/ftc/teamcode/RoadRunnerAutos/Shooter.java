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

public class Shooter {

    private DcMotorEx shooter;
    private Servo flickerServo;
    final private double upPos = 0.365;
    final private double downPos = 0.45;

    private boolean servoIsUp = false;
    private boolean inProcess = false;
    private double timeStamp = 0;

    private int lastSpeed = 0;
    public Shooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        flickerServo = hardwareMap.get(Servo.class, "flicker");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setZeroPowerBehavior(BRAKE);

        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
    }

    public void raiseServo(Turntable turntable) {
        if (turntable.getPositionId() % 2 != 0) {
            turntable.turnLeft();
        }
        if (turntable.getBallAt(turntable.getPositionId()) != null && isMoving()) {
            turntable.removeBall(turntable.getPositionId());
        }
        servoIsUp = true;
        flickerServo.setPosition(upPos);
    }

    public void lowerServo() {
        servoIsUp = false;
        flickerServo.setPosition(downPos);
    }


    public void fireAllR(Turntable turntable) {
        turntable.freeSpinR();
    }

    public void fireAllL(Turntable turntable) {
        turntable.freeSpinL();
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
