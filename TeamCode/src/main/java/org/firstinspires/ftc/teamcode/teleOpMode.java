package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class teleOpMode extends OpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;

    double power = .7;
    boolean toggle = false;

    @Override
    public void init() {

        frontLeft = hardwareMap.dcMotor.get("Front Left");
        frontRight = hardwareMap.dcMotor.get("Front Right");
        backLeft = hardwareMap.dcMotor.get("Back Left");
        backRight = hardwareMap.dcMotor.get("Back Right");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {

        frontLeft.setPower(gamepad1.left_stick_y * power);
        frontRight.setPower(gamepad1.right_stick_y * power);
        backLeft.setPower(gamepad1.left_stick_y * power);
        backRight.setPower(gamepad1.right_stick_y * power);

        if (gamepad1.dpad_left) {

            frontLeft.setPower(-power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(-power);

        }

        if (gamepad1.dpad_right) {

            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(-power);
            backRight.setPower(power);

        }

        if (gamepad1.dpad_up) {

            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);

        }

        if (gamepad1.dpad_down) {

            frontLeft.setPower(-power);
            frontRight.setPower(-power);
            backLeft.setPower(-power);
            backRight.setPower(-power);

        }

        if (gamepad1.a && !toggle) {
            toggle = true;
            if (toggle) {
                power /= 2;
                toggle = false;
            }
            else if (gamepad1.a && toggle) {
                power *= 2;
                toggle = false;
            }
        }
    }
}
