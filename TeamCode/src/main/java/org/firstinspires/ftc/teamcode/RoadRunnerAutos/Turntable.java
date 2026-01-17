package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Turntable {

    private Servo indexServo = null;
    private final double [] positions = {0.08, 0.25, 0.42, 0.59, 0.75, 0.92};

    private int currentNumBalls = 0;
    private IndexColors[] turntableBallStatus = new IndexColors[6];


    public enum IndexColors {
        PURPLE,
        GREEN
    }

    private int positionId = 0;

    public Turntable(HardwareMap hardwareMap) {
        indexServo = hardwareMap.get(Servo.class, "index");
    }

    public void turnLeft() {
        positionId--;
        updatePosition();

        IndexColors temp = turntableBallStatus[turntableBallStatus.length - 1];

        for (int i = turntableBallStatus.length - 1; i > 0; i--) {
            turntableBallStatus[i] = turntableBallStatus[i - 1];
        }

        turntableBallStatus[0] = temp;
    }

    public void turnRight() {
        positionId++;
        updatePosition();

        IndexColors temp = turntableBallStatus[0];

        for (int i = 0; i < turntableBallStatus.length - 1; i++) {
            turntableBallStatus[i] = turntableBallStatus[i + 1];
        }

        turntableBallStatus[turntableBallStatus.length - 1] = temp;
    }

    public void turnToPosition(int id) {
        int timesToTurn = id - positionId;
        for (int i = 0; i < Math.abs(timesToTurn); i++) {
            if (timesToTurn > 0) {
                turnRight();
            } else {
                turnLeft();
            }
        }
    }

    public void freeSpinR() {indexServo.setPosition(2);}
    public void freeSpinL() {indexServo.setPosition( -2);}



    public void addBall(int index, IndexColors type) {
        turntableBallStatus[index] = type;
        currentNumBalls++;
    }

    public void removeBall(int index) {
        turntableBallStatus[index] = null;
        currentNumBalls--;
    }

    public int countGreen() {
        int count = 0;
        for (IndexColors pos : turntableBallStatus) {
            if (pos == IndexColors.GREEN) {
                count++;
            }
        }
        return count;
    }

    public int countPurple() {
        int count = 0;
        for (IndexColors pos : turntableBallStatus) {
            if (pos == IndexColors.PURPLE) {
                count++;
            }
        }
        return count;
    }


    public int getNumBalls() {return currentNumBalls;}
    public int getPositionId() {return positionId;}

    public IndexColors getBallAt(int index) {return turntableBallStatus[index];}







    private void updatePosition() {
        while (positionId < 0) positionId += 6;
        while (positionId > 5) positionId -= 6;

        indexServo.setPosition(positions[positionId]);
    }

    @Override
    public String toString() {
        String output = "";
        for (int i = 0; i < turntableBallStatus.length; i++) {
            output += " Position " + i + ": " + turntableBallStatus[i];
        }

        return output;
    }

}
