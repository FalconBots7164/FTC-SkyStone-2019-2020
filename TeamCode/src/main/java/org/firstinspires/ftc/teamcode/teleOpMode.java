package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class teleOpMode extends OpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight, lift, slide;
    Servo claw, hooker;

    double powerA = .7;
    double powerB = .7;
    boolean isToggledA = false;
    boolean lockA = false;
    boolean isToggledB = false;
    boolean lockB = false;

    @Override
    public void init() {

        frontLeft = hardwareMap.dcMotor.get("Front Left");
        frontRight = hardwareMap.dcMotor.get("Front Right");
        backLeft = hardwareMap.dcMotor.get("Back Left");
        backRight = hardwareMap.dcMotor.get("Back Right");
        lift = hardwareMap.dcMotor.get("Lift");
        slide = hardwareMap.dcMotor.get("Slide");
        claw = hardwareMap.servo.get("Claw");
        hooker = hardwareMap.servo.get("Hooker");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        
        claw.setPosition(0);
        hooker.setPosition(0);

    }

    @Override
    public void loop() {

        //tank drive
        frontLeft.setPower(gamepad1.right_stick_y * -powerA);
        frontRight.setPower(gamepad1.left_stick_y * -powerA);
        backLeft.setPower(gamepad1.right_stick_y * -powerA);
        backRight.setPower(gamepad1.left_stick_y * -powerA);

        if (gamepad1.dpad_right) {

            frontLeft.setPower(-powerA);
            frontRight.setPower(-powerA);
            backLeft.setPower(powerA);
            backRight.setPower(powerA);

        }

        //strafe - completely fixed, do not touch
        if (gamepad1.dpad_left) {

            frontLeft.setPower(powerA);
            frontRight.setPower(powerA);
            backLeft.setPower(-powerA);
            backRight.setPower(-powerA);

        }

        //forward/backwards
        if (gamepad1.dpad_down) {

            frontLeft.setPower(-powerA);
            frontRight.setPower(-powerA);
            backLeft.setPower(-powerA);
            backRight.setPower(-powerA);

        }

        if (gamepad1.dpad_up) {

            frontLeft.setPower(powerA);
            frontRight.setPower(powerA);
            backLeft.setPower(powerA);
            backRight.setPower(powerA);

        }

        //speed toggleA
        if (gamepad1.a && !isToggledA && !lockA) {
            powerA = .25;
            isToggledA = true;
            lockA = true;
        }
        else if (gamepad1.a && isToggledA && !lockA) {
            powerA = .7;
            isToggledA = false;
            lockA = true;
        }
        else if (!gamepad1.a && lockA) {
            lockA = false;
        }

        //gamepad2
        slide.setPower(gamepad2.right_stick_y * powerB * .65);
        lift.setPower(gamepad2.left_stick_y * powerB * .3);

        if(gamepad2.a && !isToggledB && !lockB) {
            powerB = .25;
            isToggledB = true;
            lockB = true;
        }
        else if (gamepad2.a && isToggledB && !lockB) {
            powerB = .7;
            isToggledB = false;
            lockB = true;
        }
        else if (!gamepad2.a && lockB) {
            lockB = false;
        }


        //servo
        if (gamepad2.y) {
            claw.setPosition(.5);
        }
        else {
            claw.setPosition(0);
        }

        if (gamepad2.x) {
            hooker.setPosition(.5);
        }
        else {
            hooker.setPosition(0);
        }
        telemetry.addData("Gamepad 1 Toggle:", isToggledA);
        telemetry.addData("Gamepad 2 Toggle:", isToggledB);
    }
}
