package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class autoTest extends LinearOpMode {

    //VARIABLES
    final static double pi = 3.1415;
    final static double ticksPerRevolution = 1100;
    final static double wheelDiameter = 4;
    final static double wheelCircumference = (wheelDiameter * pi);
    final static double encoderTicksPerInch = (ticksPerRevolution / wheelCircumference);


    //device variables
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    //METHODS

    //methods for state machine
    public void changeState(programSteps newState) {
        step = newState;
    }

    public programSteps getState() {
        return step;
    }

    //movement methods
    public void moveForward(double power, double inches) {
        if (opModeIsActive()) {
            int frontLeftPosition = frontLeft.getCurrentPosition() + (int) (inches * encoderTicksPerInch);
            frontLeft.setTargetPosition(frontLeftPosition);
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);

        }
    }

    //state machine
    public enum programSteps {
        stepOne,
        stepTwo,
        stepThree
    }

    programSteps step;



    @Override
    public void runOpMode() throws InterruptedException {
        //hardware mapping


        //initialize encoders

        //initialize vuforia


        //autonomous begins after this
        waitForStart();
        while (opModeIsActive()) {
            switch (step) {
                case stepOne:

                    break;
                case stepTwo:

                    break;
                case stepThree:

                    break;
            }
        }


    }
}
