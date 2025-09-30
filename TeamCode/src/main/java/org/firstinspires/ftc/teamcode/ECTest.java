package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class ECTest extends LinearOpMode {

    private DcMotor Front_Left;
    private DcMotor Front_Right;
    private DcMotor Back_Left;
    private DcMotor Back_Right;
    private int Front_LeftPOS;
    private int Front_RightPOS;
    private int Back_LeftPOS;
    private int Back_RightPOS;
    private DcMotor Intake;
    private DcMotor Indexer;
    private int WobblerPOS;
    private DcMotor Shooter;
    private DcMotor Wobbler;
    private Servo Clamp;

    @Override
    public void runOpMode() throws InterruptedException {

        Front_Left = hardwareMap.dcMotor.get("Front_Left");
        Front_Right = hardwareMap.dcMotor.get("Front_Right");
        Back_Left = hardwareMap.dcMotor.get("Back_Left");
        Back_Right = hardwareMap.dcMotor.get("Back_Right");
        Intake = hardwareMap.dcMotor.get("Intake");
        Indexer = hardwareMap.dcMotor.get("Indexer");
        Shooter = hardwareMap.dcMotor.get("Shooter");
        Wobbler = hardwareMap.dcMotor.get("Wobbler");
        Clamp = hardwareMap.servo.get("Clamp");

        Front_Left.setDirection(DcMotorSimple.Direction.REVERSE);
        Back_Left.setDirection(DcMotorSimple.Direction.REVERSE);

        Front_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Wobbler.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();


            /*while (opModeIsActive()) {
            Front_Left.setPower(-0.75);
            Front_Right.setPower(-0.5);
            Back_Left.setPower(-0.75);
            Back_Right.setPower(-0.5);
            sleep(500);

//stop
            Front_Left.setPower(0);
            Front_Right.setPower(0);
            Back_Left.setPower(0);
            Back_Right.setPower(0);
            sleep(75000);
            */

        Clamp.setPosition(0.0);
        Wobbler.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Wobbler.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Wobbler.setTargetPosition(1800);
        Wobbler.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        WobblerPOS = Wobbler.getCurrentPosition();

        while (opModeIsActive() && WobblerPOS < 1800) {

            Wobbler.setPower(0.3);

            WobblerPOS = Wobbler.getCurrentPosition();
        }
//stop
        Front_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Front_Left.setPower(0);
        Front_Right.setPower(0);
        Back_Right.setPower(0);
        Back_Left.setPower(0);

        Front_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(1000);

//move forward
        Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Front_Left.setTargetPosition(500);
        Front_Right.setTargetPosition(-500);
        Back_Right.setTargetPosition(-500);
        Back_Left.setTargetPosition(500);

        Front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Front_LeftPOS = Front_Left.getCurrentPosition();
        Front_RightPOS = Front_Right.getCurrentPosition();
        Back_RightPOS = Back_Right.getCurrentPosition();
        Back_LeftPOS = Back_Left.getCurrentPosition();

        while (opModeIsActive() && Front_LeftPOS < 500 && Front_RightPOS > -500 && Back_RightPOS > -500 && Back_LeftPOS < 500) {


            Front_Left.setPower(1);
            Front_Right.setPower(1);
            Back_Right.setPower(1);
            Back_Left.setPower(1);

            Front_LeftPOS = Front_Left.getCurrentPosition();
            Front_RightPOS = Front_Right.getCurrentPosition();
            Back_RightPOS = Back_Right.getCurrentPosition();
            Back_LeftPOS = Back_Left.getCurrentPosition();
        }


//stop
        Front_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Front_Left.setPower(0);
        Front_Right.setPower(0);
        Back_Right.setPower(0);
        Back_Left.setPower(0);

        Front_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(3000);

//strafe left
        /*Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Front_Left.setTargetPosition(-400);
        Front_Right.setTargetPosition(-400);
        Back_Right.setTargetPosition(400);
        Back_Left.setTargetPosition(400);

        Front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Front_LeftPOS = Front_Left.getCurrentPosition();
        Front_RightPOS = Front_Right.getCurrentPosition();
        Back_RightPOS = Back_Right.getCurrentPosition();
        Back_LeftPOS = Back_Left.getCurrentPosition();

        while (opModeIsActive() && Front_LeftPOS > -200 && Front_RightPOS > -200 && Back_RightPOS < 200 && Back_LeftPOS < 200) {

            Front_Left.setPower(1);
            Front_Right.setPower(1);
            Back_Right.setPower(1);
            Back_Left.setPower(1);

            Front_LeftPOS = Front_Left.getCurrentPosition();
            Front_RightPOS = Front_Right.getCurrentPosition();
            Back_RightPOS = Back_Right.getCurrentPosition();
            Back_LeftPOS = Back_Left.getCurrentPosition();
        }

//stop
        Front_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Front_Left.setPower(0);
        Front_Right.setPower(0);
        Back_Right.setPower(0);
        Back_Left.setPower(0);

        Front_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(3000); */
        


        Shooter.setPower (-1.0);
        sleep(3000);

        Shooter.setPower (-1.0);
        Indexer.setPower (-1.0);
        sleep(3000);
        
        Shooter.setPower (-1.0);
        Indexer.setPower (-1.0);
        Intake.setPower (1.0);
        sleep(4000);
        
        Shooter.setPower (0);
        Indexer.setPower (0);
        Intake.setPower (0);
        sleep(4000);


//stop
        Front_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Front_Left.setPower(0);
        Front_Right.setPower(0);
        Back_Right.setPower(0);
        Back_Left.setPower(0);

        Front_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(2000);



//move forward
        Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Front_Left.setTargetPosition(500);
        Front_Right.setTargetPosition(-500);
        Back_Right.setTargetPosition(-500);
        Back_Left.setTargetPosition(500);

        Front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Front_LeftPOS = Front_Left.getCurrentPosition();
        Front_RightPOS = Front_Right.getCurrentPosition();
        Back_RightPOS = Back_Right.getCurrentPosition();
        Back_LeftPOS = Back_Left.getCurrentPosition();

        while (opModeIsActive() && Front_LeftPOS < 500 && Front_RightPOS > -500 && Back_RightPOS > -500 && Back_LeftPOS < 500) {

            Front_Left.setPower(1);
            Front_Right.setPower(0.8);
            Back_Right.setPower(0.8);
            Back_Left.setPower(1);

            Front_LeftPOS = Front_Left.getCurrentPosition();
            Front_RightPOS = Front_Right.getCurrentPosition();
            Back_RightPOS = Back_Right.getCurrentPosition();
            Back_LeftPOS = Back_Left.getCurrentPosition();
        }
//stop
        Front_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Front_Left.setPower(0);
        Front_Right.setPower(0);
        Back_Right.setPower(0);
        Back_Left.setPower(0);

        Front_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(1000);
        
        Clamp.setPosition(0.5);
        
//stop
        Front_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Front_Left.setPower(0);
        Front_Right.setPower(0);
        Back_Right.setPower(0);
        Back_Left.setPower(0);

        Front_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(1000);
        
//move backward
        Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Front_Left.setTargetPosition(-200);
        Front_Right.setTargetPosition(200);
        Back_Right.setTargetPosition(200);
        Back_Left.setTargetPosition(-200);

        Front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Front_LeftPOS = Front_Left.getCurrentPosition();
        Front_RightPOS = Front_Right.getCurrentPosition();
        Back_RightPOS = Back_Right.getCurrentPosition();
        Back_LeftPOS = Back_Left.getCurrentPosition();

        while (opModeIsActive() && Front_LeftPOS > -200 && Front_RightPOS < 200 && Back_RightPOS < 200 && Back_LeftPOS > -200) {


            Front_Left.setPower(1);
            Front_Right.setPower(1);
            Back_Right.setPower(1);
            Back_Left.setPower(1);

            Front_LeftPOS = Front_Left.getCurrentPosition();
            Front_RightPOS = Front_Right.getCurrentPosition();
            Back_RightPOS = Back_Right.getCurrentPosition();
            Back_LeftPOS = Back_Left.getCurrentPosition();
        }

        
        
//stop
        Front_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Front_Left.setPower(0);
        Front_Right.setPower(0);
        Back_Right.setPower(0);
        Back_Left.setPower(0);

        Front_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(1000000);
        
        /*Shooter.setPower (-1.0);
        sleep(2000);

        Shooter.setPower (-1.0);
        Indexer.setPower (-1.0);
        Intake.setPower (1.0);
        sleep(5000);

        
//park on launch line
        Front_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Front_Left.setPower(0);
        Front_Right.setPower(0);
        Back_Right.setPower(0);
        Back_Left.setPower(0);

        Front_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(10000);
        */






    }
}



