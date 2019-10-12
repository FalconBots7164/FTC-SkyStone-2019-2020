package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class testTeleOP extends OpMode {

    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;

    @Override
    public void init() {
       motor1 = hardwareMap.dcMotor.get("Motor 1");
       motor2 = hardwareMap.dcMotor.get("Motor 2");
       motor3 = hardwareMap.dcMotor.get("Motor 3");
       motor4 = hardwareMap.dcMotor.get("Motor 4");
    }

    @Override
    public void loop() {
        //motor1.setPower(gamepad1.left_bumper);
        if (gamepad1.left_bumper == true) {
            motor1.setPower(-1);
            motor2.setPower(-1);
            motor3.setPower(1);
            motor4.setPower(1);
        }
        if (gamepad1.right_bumper == true) {
            motor1.setPower(1);
            motor2.setPower(1);
            motor3.setPower(-1);
            motor4.setPower(-1);
        }
        motor1.setPower(gamepad1.left_stick_y * -1);
        motor2.setPower(gamepad1.right_stick_y);
        motor3.setPower(gamepad1.left_stick_y * -1);
        motor4.setPower(gamepad1.right_stick_y);
    }
}
