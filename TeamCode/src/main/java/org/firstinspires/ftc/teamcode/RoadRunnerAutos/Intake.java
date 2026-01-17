package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intake;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void setPower(double power) {
        intake.setPower(power);
    }

    public boolean autoIntake(Camera camera, Turntable turntable) {
        int indexToAddBall;
        switch (turntable.getNumBalls()) {
            case 0:
                indexToAddBall = 1;
                break;
            case 1:
                indexToAddBall = 3;
                break;
            case 2:
                indexToAddBall = 5;
                break;
            default:
                return false;
        }
        turntable.turnToPosition(indexToAddBall);

        if (camera.ballDetected()) {
            turntable.addBall(indexToAddBall, camera.getBallColor());
            turntable.turnRight();
            turntable.turnRight();
        }
        return true;
    }

    public void stopIntake() {
        intake.setPower(0);
    }


}
