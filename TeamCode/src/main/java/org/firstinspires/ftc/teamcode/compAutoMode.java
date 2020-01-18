package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;
import java.util.*;

import java.util.Scanner;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@Autonomous
public class compAutoMode extends LinearOpMode {

    //VARIABLES

    //Constants for encoders
    private final static double ticksPerRevolution = 1120; //rev hex motor
    private final static double wheelDiameter = 4.0;
    private static double wheelGearRatio = 1.0;
    private final static double encoderTicksPerInch = (ticksPerRevolution * wheelGearRatio) / (wheelDiameter * Math.PI);

    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;


    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    //Vuforia
    private static final String VUFORIA_KEY = "ARZTw9L/////AAABmRmjM+ct9kkHs003U+Tv11d8yVGXCaI6tYp7lTtQqnvd4p6/5LdopGJP6+8imxm2HZdj88a0AZu48Q7DeqbAtiIDf/ZAcOFqFmlKwbFrRzLfiJMXOcsLL4KkmKwrZwxXDWsLBwchPrj4uZGnoeg3PVLHo4bVVeYU7wkOFlR146ZEbuhvHS0ml+HFeOCAsIwW4B/joj9mdDMkEvFhosKv8ZzLUfAThvTlMNp6Me/QPv4qu/4fXlFkHbrgGRjT2dau0FHCaU8j26MYIJdgIt+QUDmN/xxG9QlFDHJZjkeid3CnmLsqOxbp7HbLFI8pv7TRWVY68RIjZIs1NcSTakz6RVyUl3lt555JNtM7vVMYEzU9";
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    private OpenGLMatrix lastLocation = null;
    private boolean skystoneVisible = false;
    private boolean navIsVisible = false;
    private VuforiaLocalizer vuforia = null;
    private WebcamName webcamName = null;

    //TensorFlow
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private TFObjectDetector tfod;


    //device variables
    DcMotor frontLeft, frontRight, backLeft, backRight, lift, slide;
    Servo claw;

    //gyro variables
    double globalAngle;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();

    //METHODS

    //methods for state machine
    private void changeState(programSteps newState) {
        step = newState;
    }

    public void resetState() {
        changeState(programSteps.findSkystone);
    }

    public programSteps getState() {
        return step;
    }

    public void moveForward(double power, double inches) {

        if (inches != 0) {

            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //creating the variables that will tell the encoder how many ticks to travel
            int frontLeftTarget = frontLeft.getCurrentPosition() + (int) (-inches * encoderTicksPerInch);
            int frontRightTarget = frontRight.getCurrentPosition() + (int) (-inches * encoderTicksPerInch);
            int backLeftTarget = backLeft.getCurrentPosition() + (int) (-inches * encoderTicksPerInch);
            int backRightTarget = backRight.getCurrentPosition() + (int) (-inches * encoderTicksPerInch);

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
            while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                telemetry.addData("Front Left Ticks", frontLeft.getCurrentPosition());
                telemetry.addData("Front Right Ticks", frontRight.getCurrentPosition());
                telemetry.addData("Back Left Ticks", backLeft.getCurrentPosition());
                telemetry.addData("Back Right Ticks", backRight.getCurrentPosition());
                telemetry.update();
            }

            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        } else if (inches == 0) {
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);

            return;
        }
    }

    public void moveHorizontal(double power, double inches) {

        if (inches != 0 && inches != -1) {

            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //creating the variables that will tell the encoder how many ticks to travel
            int frontLeftTarget = frontLeft.getCurrentPosition() + (int) (inches * encoderTicksPerInch);
            int frontRightTarget = frontRight.getCurrentPosition() + (int) (inches * encoderTicksPerInch);
            int backLeftTarget = backLeft.getCurrentPosition() + (int) (-inches * encoderTicksPerInch);
            int backRightTarget = backRight.getCurrentPosition() + (int) (-inches * encoderTicksPerInch);

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
            while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                telemetry.addData("Front Left Ticks", frontLeft.getCurrentPosition());
                telemetry.addData("Front Right Ticks", frontRight.getCurrentPosition());
                telemetry.addData("Back Left Ticks", backLeft.getCurrentPosition());
                telemetry.addData("Back Right Ticks", backRight.getCurrentPosition());
                telemetry.update();
            }

            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //right
        } else if (inches == 0) {

            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            frontLeft.setPower(-power);
            frontRight.setPower(-power);
            backLeft.setPower(power);
            backRight.setPower(power);

            return;
            //left
        } else if (inches == -1) {

            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(-power);
            backRight.setPower(-power);

            return;
        }

    }

    public void moveSlide(double power, double inches) {

        if (opModeIsActive() && inches != 0) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            int slideTarget = slide.getCurrentPosition() + (int) (-inches * encoderTicksPerInch);
            slide.setTargetPosition(slideTarget);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(power);
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            while (opModeIsActive() && slide.isBusy()) {
                telemetry.addData("Slide Ticks", slide.getCurrentPosition());
                telemetry.update();
            }
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else if (opModeIsActive() && inches == 0) {
            slide.setPower(power);
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void moveLift(double power, double inches) {

        if (opModeIsActive() && inches != 0) {
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            int liftTarget = lift.getCurrentPosition() + (int) (-inches * encoderTicksPerInch);
            lift.setTargetPosition(liftTarget);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(power);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while (opModeIsActive() && lift.isBusy()) {
                telemetry.addData("Lift Ticks", slide.getCurrentPosition());
                telemetry.update();
            }
        } else if (opModeIsActive() && inches == 0) {
            lift.setPower(power);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void moveClaw(double position) {
        claw.setPosition(position);
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

    }

    public void resetAngle() {

        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;

    }

    public double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        //Rotation only returned from -180 to 180 and will begin to count backwards after the threshold.
        //Ex: Going beyond 180 will start counting backwards towards -179, -178, -177, etc, which throws off the rotations.
        //So, if rotation is greater than 180 or less than -180,
        //it must be changed to be within the threshold -180 <= x <= 180.
        //The delta rotation is then added to the global rotation.
        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }

    //positive degrees rotates left, negative rotates right.
    public void rotateRobot(double power, int degrees) {

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        resetAngle();

        if (degrees < 0) {
            //Right
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(power);
            backRight.setPower(-power);
        } else if (degrees > 0) {
            //Left
            frontLeft.setPower(-power);
            frontRight.setPower(power);
            backLeft.setPower(-power);
            backRight.setPower(power);
        }

        //exit method if there is no need to rotate
        else {
            return;
        }

        //right
        if (degrees < 0) {
            while (opModeIsActive() && getAngle() == 0) {
                telemetry.addData("Degrees", lastAngles.firstAngle);
                telemetry.addData("Global Angle", globalAngle);
                telemetry.update();
            }

            while (opModeIsActive() && getAngle() > degrees) {
                telemetry.addData("Degrees", lastAngles.firstAngle);
                telemetry.addData("Global Angle", globalAngle);
                telemetry.update();
            }
            //over-rot check
            //hopefully can combat the over-rotating, by rotating in the other direction.
            if (getAngle() < degrees) {
                while (opModeIsActive() && getAngle() < degrees) {
                    frontLeft.setPower(-power * .25);
                    frontRight.setPower(power * .25);
                    backLeft.setPower(-power * .25);
                    backRight.setPower(power * .25);
                }
            }
        }

        //left
        else {
            while (opModeIsActive() && getAngle() < degrees) {
                telemetry.addData("Degrees", lastAngles.firstAngle);
                telemetry.addData("Global Angle", globalAngle);
                telemetry.update();
            }
            //over-rot check
            if (getAngle() > degrees) {
                while (opModeIsActive() && getAngle() > degrees) {
                    frontLeft.setPower(power * .25);
                    frontRight.setPower(-power * .25);
                    backLeft.setPower(power * .25);
                    backRight.setPower(-power * .25);
                }
            }
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        resetAngle();

    }

    public void straightenRobot(double power) {

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (getAngle() < 0) {
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(power);
            backRight.setPower(-power);

            while (opModeIsActive() && getAngle() < 0) {
            }

        } else if (getAngle() > 0) {
            frontLeft.setPower(-power);
            frontRight.setPower(power);
            backLeft.setPower(-power);
            backRight.setPower(power);

            while (opModeIsActive() && getAngle() > 0) {
            }

        } else {
            return;
        }
    }

    //state machine
    public enum programSteps {
        findColor,
        findSkystone,
        grabSkystone,
        goUnderBridge,
        placeSkystone,
        movePlate,
        stop
    }

    public enum Color {
        RED,
        BLUE
    }

    programSteps step;
    Color color;

    @Override
    public void runOpMode() throws InterruptedException {

        //hardware mapping
        frontLeft = hardwareMap.dcMotor.get("Front Left");
        frontRight = hardwareMap.dcMotor.get("Front Right");
        backLeft = hardwareMap.dcMotor.get("Back Left");
        backRight = hardwareMap.dcMotor.get("Back Right");
        lift = hardwareMap.dcMotor.get("Lift");
        slide = hardwareMap.dcMotor.get("Slide");
        claw = hardwareMap.servo.get("Claw");
        imu = hardwareMap.get(BNO055IMU.class, "IMU");
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

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

        //init gyro
        telemetry.addData("Status", "Initializing Gyro");
        telemetry.update();

        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();

        parametersIMU.mode = BNO055IMU.SensorMode.IMU;
        parametersIMU.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.loggingEnabled = false;
        parametersIMU.loggingEnabled = true;
        parametersIMU.loggingTag = "IMU";
        imu.initialize(parametersIMU);

        telemetry.addData("Status", "Gyro Ready");
        telemetry.update();

        //init vuforia
        telemetry.addData("Status", "Initializing Vuforia");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parametersVuf = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parametersVuf.vuforiaLicenseKey = VUFORIA_KEY;
        parametersVuf.cameraName = webcamName;
        parametersVuf.cameraDirection = CAMERA_CHOICE;
        vuforia = ClassFactory.getInstance().createVuforia(parametersVuf);
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        //Model data from source code????
        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1"); //red
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2"); //blue
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        final int CAMERA_FORWARD_DISPLACEMENT = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parametersVuf.cameraDirection);
        }

        telemetry.addData("Status", "Vuforia Ready");
        telemetry.update();

        telemetry.addData("Status", "Initializing TensorFlow");
        telemetry.update();

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

        telemetry.addData("Status", "TensorFlow Ready");
        telemetry.update();

        resetAngle();

        telemetry.addData("Status", "Everything Ready");
        telemetry.update();

        waitForStart();

        resetState();
//For bot to know what side its on
// Find certain pic -- tell if its red
// else its blue
//   ANIME GIRLS O o O
        while (opModeIsActive()) {
            switch (step) {
                case findColor:
                    targetsSkyStone.activate();
                    navIsVisible = false;
                    moveForward(.5, 4);
                    rotateRobot(1, -180);
                    while (!navIsVisible) {
                        if (((VuforiaTrackableDefaultListener) front1.getListener()).isVisible()) {
                            color = Color.RED;
                            navIsVisible = true;
                        } else if (((VuforiaTrackableDefaultListener) front2.getListener()).isVisible()) {
                            color = Color.BLUE;
                            navIsVisible = true;
                        }
                        changeState(programSteps.findSkystone);
                        break;
                    }
                    break;
                case findSkystone:
                    color = Color.RED;
                    targetsSkyStone.activate();

                    moveForward(.5, 8);
                    if (color == Color.RED) {
                        moveHorizontal(.25, 0);
                    } else if (color == Color.BLUE) {
                        moveHorizontal(.25, -1);
                    }

                    skystoneVisible = false;
                    while (opModeIsActive() && !skystoneVisible) {
                        if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
                            stopRobot();
                            skystoneVisible = true;
                            break;
                        }
                    }

                    //once target is found, position robot so that the webcam and skystone are perpendicular.
                    while (opModeIsActive() && skystoneVisible) {
                        //create a matrix for the position of the skystone which will be converted into vector quantities.
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                        //declaration of vector
                        VectorF position = lastLocation.getTranslation();
                        if (position.get(1) / mmPerInch > 0) {
                            moveHorizontal(.25, 0);
                        } else if (position.get(1) / mmPerInch < 0) {
                            moveHorizontal(.25, -1);
                        }

                        if (position.get(1) / mmPerInch <= 0.025f || position.get(1) / mmPerInch >= -0.025f) {
                            stopRobot();
                            break;
                        }
                    }
                    targetsSkyStone.deactivate();
                    changeState(programSteps.grabSkystone);
                    break;
                case grabSkystone:
                    //once perpendicular, adjust robot to grab the block.
                    moveSlide(1, -6);
                    moveForward(.5, 7);
                    moveHorizontal(.5, 4);
                    moveClaw(.7);
                    straightenRobot(.2);
//                    rotateRobot(.2, -5);
                    moveForward(.2, 8);
                    moveClaw(-.2);
//                    sleep(500);
//                    moveSlide(.4, 6);
                    moveForward(.5, -12);
                    changeState(programSteps.goUnderBridge);
                    break;
                case goUnderBridge:
                    moveForward(.5, -13);
                    rotateRobot(1, -90);
                    moveForward(1, 26);
                    changeState(programSteps.placeSkystone);
                    break;

                case placeSkystone:
                    changeState(programSteps.movePlate);
                    break;

                case movePlate:
                    changeState(programSteps.stop);
                    break;

                case stop:
                    stopRobot();
                    break;

                default:

                    telemetry.addData("Error", "Something went wrong");
                    telemetry.update();
                    stopRobot();
                    break;
            }
        }
    }
}
