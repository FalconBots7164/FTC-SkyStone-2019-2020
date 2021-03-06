package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class autoTest extends LinearOpMode {

    //VARIABLES
    final static double pi = 3.1415;
    final static double ticksPerRevolution = 1120; //rev hex motor
    final static double wheelDiameter = 4.0;
    final static double wheelGearRatio = 1.0;
    final static double encoderTicksPerInch = (ticksPerRevolution * wheelGearRatio) / (wheelDiameter * pi);

    //device variables
    DcMotor frontLeft, frontRight, backLeft, backRight;

    //METHODS

    //methods for state machine
    private void changeState(programSteps newState) {
        step = newState;
    }

    public void resetState() {
        changeState(programSteps.stepOne);
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
            int frontLeftTarget = frontLeft.getCurrentPosition() + (int) (inches * encoderTicksPerInch);
            int frontRightTarget = frontRight.getCurrentPosition() + (int) (inches * encoderTicksPerInch);
            int backLeftTarget = backLeft.getCurrentPosition() + (int) (inches * encoderTicksPerInch);
            int backRightTarget = backRight.getCurrentPosition() + (int) (inches * encoderTicksPerInch);

            //setting the aforementioned variables as the target position
            frontLeft.setTargetPosition(frontLeftTarget);
            frontRight.setTargetPosition(frontRightTarget);
            backLeft.setTargetPosition(backLeftTarget);
            backRight.setTargetPosition(backRightTarget);

            //telling the motors to start moving towards target
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //setting the power for the motors to move while travelling towards target
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);

            //telling the motors to brake when power is zero (when target is reached).
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //telemetries to tell the user how many ticks the encoder is travelling
            while (opModeIsActive() || frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) {
                telemetry.addData("Front Left Ticks", frontLeft.getCurrentPosition());
                telemetry.addData("Front Right Ticks", frontRight.getCurrentPosition());
                telemetry.addData("Back Left Ticks", backLeft.getCurrentPosition());
                telemetry.addData("Back Right Ticks", backRight.getCurrentPosition());

                telemetry.addData("Front Left Target", frontLeftTarget);
                telemetry.addData("Front Right Target", frontRightTarget);
                telemetry.addData("Back Left Target", backLeftTarget);
                telemetry.addData("Back Right Target", backRightTarget);
                telemetry.update();
            }
            
//            //make the motors stop once the target is reached.
//            frontLeft.setPower(0);
//            frontRight.setPower(0);
//            backLeft.setPower(0);
//            backRight.setPower(0);
            
        }
    }

    public void stopRobot() {
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //state machine
    public enum programSteps {
        stepOne,
        stepTwo,
        stepThree,
        stop
    }

    programSteps step;

    @Override
    public void runOpMode() throws InterruptedException {

        //hardware mapping
        frontLeft = hardwareMap.dcMotor.get("Front Left");
        frontRight = hardwareMap.dcMotor.get("Front Right");
        backLeft = hardwareMap.dcMotor.get("Back Left");
        backRight = hardwareMap.dcMotor.get("Back Right");

        //make all motor directions uniform
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


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

        resetState();

        while (opModeIsActive()) {
            switch (step) {
                case stepOne:
                    changeState(programSteps.stepTwo);
                    telemetry.addData("Step", getState());
                    telemetry.update();
                    break;

                case stepTwo:
                    moveForward(.5, 12);
                    changeState(programSteps.stepThree);
                    telemetry.addData("Step", getState());
                    telemetry.update();
                    break;

                case stepThree:
  //                  moveForward(.25, 6);
                    changeState(programSteps.stop);
                    telemetry.addData("Step", getState());
                    telemetry.update();
                    break;

                case stop:
//                    stopRobot();
                    telemetry.addData("Step", getState());

                    break;

                default:

                    telemetry.addData("Error","Something went wrong");
                    telemetry.update();

                    break;
            }
        }
    }
}
