package org.firstinspires.ftc.teamcode;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class test2 extends LinearOpMode {

    //This program works in Euler angles, and since the robot is parallel to the ground,
    //we want it to rotate around the z-axis (Yaw, or Psi), which is perpendicular to the ground.

    //Variables
    double globalAngle, correction;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();

    //Methods
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
        }

        else if (deltaAngle > 180 ) {
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }

    public double correctDirection() {

        //This method does not involve rotation, rather it involves using the IMU to make
        //sure the robot is moving in a perfectly straight line.
        //If the robot is not moving straight, it will add power to the side that needs adjusting
        //until it is straight once more. This can be very important for autonomous to ensure accurate movement,
        //so it is very important this is adjusted to work with encoders, and see if this can be implemented
        //with the mecanum wheels moving side-to-side.

        double correction, angle, gain = .10;

        angle = getAngle();

        //if straight, no angle change is needed.
        if (angle == 0) {
            correction = 0;
        }
        //else, set correction to negative angle, so it can adjust properly.
        else {
            correction = -angle;
        }

        correction = correction * gain;

        return correction;
    }


    //positive degrees rotates left, negative rotates right.
    public void rotateRobot(double power, int degrees) {

        resetAngle();

        if (degrees < 0) {
            frontLeft.setPower(-power);
            frontRight.setPower(power);
            backLeft.setPower(-power);
            backRight.setPower(power);
        }

        else if (degrees > 0) {
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(power);
            backRight.setPower(-power);
        }

        //exit method if there is no need to rotate
        else {
            return;
        }

        //right
        if (degrees < 0) {
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }

        //left
        else {
            while (opModeIsActive() && getAngle() < degrees) {}
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        sleep(1000);

        resetAngle();

    }

    public void moveForward(double power, long time) {

        frontLeft.setPower(power - correction);
        frontRight.setPower(power + correction);
        backLeft.setPower(power - correction);
        backRight.setPower(power + correction);

        if (time > 3000) {

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            return;
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        frontLeft = hardwareMap.dcMotor.get("Front Left");
        frontRight = hardwareMap.dcMotor.get("Front Right");
        backLeft = hardwareMap.dcMotor.get("Back Left");
        backRight = hardwareMap.dcMotor.get("Back Right");
        imu = hardwareMap.get(BNO055IMU.class,"IMU");
        //IMPORTANT: IMU must be set on I2C channel 0, port 0, and nothing else

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //initializing IMU
        imu.initialize(parameters);

        telemetry.addData("IMU","Initializing...");
        telemetry.addData("Status:",imu.getCalibrationStatus().toString());
        telemetry.update();

        //initializing takes time, so make sure that it is properly calibrated before it starts

        //wait to print ready until the initialization is finished
//        while (!imu.isGyroCalibrated()) {}

            telemetry.addData("IMU", "Ready");
            telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            correction = correctDirection();
            moveForward(.5, 2000);
            sleep(1000);
            rotateRobot(.5, 90);

            while (opModeIsActive()) {
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.update();
            }
        }
    }
}
