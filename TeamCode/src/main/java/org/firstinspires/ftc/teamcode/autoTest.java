package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class autoTest extends LinearOpMode {

    //VARIABLES
    final static double pi = 3.1415;
    final static double ticksPerRevolution = 1100; //placeholder, will need to be changed. rev hex motors are 40:1, so it could possibly be 1200/1100 again.
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
    //Consider using gyro sensor for more accurate turning rather than encoders, save the pain of finding the equation for ticks per degree.


    //encoder testing, will need to wait until we have four rev hex motors however.
    public void moveForward(double power, double inches) {

        if (opModeIsActive()) {

            //creating the variables that will tell the encoder how many ticks to travel
            int frontLeftPosition = frontLeft.getCurrentPosition() + (int) (inches * encoderTicksPerInch);
            int frontRightPosition = frontRight.getCurrentPosition() + (int) (inches * encoderTicksPerInch);
            int backLeftPosition = backLeft.getCurrentPosition() + (int) (inches * encoderTicksPerInch);
            int backRightPosition = backRight.getCurrentPosition() + (int) (inches * encoderTicksPerInch);

            //setting the aforementioned variables as the target position
            frontLeft.setTargetPosition(frontLeftPosition);
            frontRight.setTargetPosition(frontRightPosition);
            backLeft.setTargetPosition(backLeftPosition);
            backRight.setTargetPosition(backRightPosition);

            //telling the motors to start moving towards target
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //setting the power for the motors to move while travelling towards target
            frontLeft.setPower(-power);
            frontRight.setPower(-power);
            backLeft.setPower(power);
            backRight.setPower(power);

            //telling the motors to brake when power is zero (when target is reached).
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //telemetries to tell the user how many ticks the encoder is travelling
            while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                telemetry.addData("Front Left Ticks", frontLeft.getCurrentPosition());
                telemetry.addData("Front Right Ticks", frontRight.getCurrentPosition());
                telemetry.addData("Back Left Ticks", backLeft.getCurrentPosition());
                telemetry.addData("Back Right Ticks", backRight.getCurrentPosition());
                telemetry.update();
            }
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
        frontLeft = hardwareMap.dcMotor.get("Front Left");
        frontRight = hardwareMap.dcMotor.get("Front Right");
        backLeft = hardwareMap.dcMotor.get("Back Left");
        backRight = hardwareMap.dcMotor.get("Back Right");

        //initialize encoders
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Encoders Ready");
        telemetry.update();


        //initialize vuforia
        //Placeholder, need to get vuforia working first with new pictures.


        //autonomous begins after this
        waitForStart();

        while (opModeIsActive()) {
            switch (step) {
                case stepOne:

                    moveForward(.7, 12);

                    break;
                case stepTwo:


                    break;
                case stepThree:


                    break;
                    
                default:
                    
                    telemetry.addData("Error","Something went wrong");
                    telemetry.update();
                    
                    break;
            }
        }
    }
}
