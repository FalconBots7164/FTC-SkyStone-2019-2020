package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class test2 extends OpMode {

    DcMotor motor1;


    @Override
    public void init() {

        motor1 = hardwareMap.dcMotor.get("Motor 1");

    }

    @Override
    public void loop() {

        motor1.setPower(gamepad1.right_stick_y);

    }
}
