package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Robot extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;

    public Robot() {}

    public void initializeHardware() {

        telemetry.addData("Hardware", "Initializing...");
        telemetry.update();

        frontLeft = hardwareMap.dcMotor.get("Front Left");
        frontRight = hardwareMap.dcMotor.get("Front Right");
        backLeft = hardwareMap.dcMotor.get("Back Left");
        backRight = hardwareMap.dcMotor.get("Back Right");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Hardware", "Done");
        telemetry.update();

    }

    public void initializeEncoders() {

    }

    public void initializeIMU() {

    }

    @Override
    public void runOpMode() throws InterruptedException {

    }


}
