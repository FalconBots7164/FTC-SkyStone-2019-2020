package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static android.os.SystemClock.sleep;

@TeleOp
public class teleOpModeDAM extends OpMode {
//damian
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    double power = .7;
  //boolean toggle = false;

    double[] change = {.35, .7};

    long l;

    @Override
    public void init() {

        frontLeft = hardwareMap.dcMotor.get("Front Left");
        frontRight = hardwareMap.dcMotor.get("Front Right");
        backLeft = hardwareMap.dcMotor.get("Back Left");
        backRight = hardwareMap.dcMotor.get("Back Right");

    }

    @Override
    public void loop() {

        frontLeft.setPower(gamepad1.left_stick_y * power);
        frontRight.setPower(gamepad1.right_stick_y * -power);
        backLeft.setPower(gamepad1.left_stick_y * power);
        backRight.setPower(gamepad1.right_stick_y * -power);

        if (gamepad1.dpad_left || gamepad1.left_bumper) { /* Strafe Left???? */

            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);

        }

        if (gamepad1.dpad_right) {

            frontLeft.setPower(-power);
            frontRight.setPower(-power);
            backLeft.setPower(-power);
            backRight.setPower(-power);

        }

        if (gamepad1.dpad_up) {

            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(power);
            backRight.setPower(-power);
        }

        if (gamepad1.dpad_down) {
            frontLeft.setPower(-power);
            frontRight.setPower(power);
            backLeft.setPower(-power);
            backRight.setPower(power);
        }

        /////////////////////////////////////////

        while (gamepad1.a) {
            //sleep(.0001);
            power = change[0];
            telemetry.addData("ACTIVE.", power);        /* Just prints the current power NOT an array value*/
            if (!gamepad1.a) {
                power = change[1];
                telemetry.addData("INACTIVE.", power);  /* same */
                break;
            }
            else {
                telemetry.addData("ERR"," ERR");
            }

        }

        telemetry.addData("Robot Version","v1.03");
        telemetry.addData("frontRight Power", frontRight.getPower());
        telemetry.addData("backRight Power", backRight.getPower());
        telemetry.addData("frontLeft Power", frontLeft.getPower());
        telemetry.addData("backLeft Power", backLeft.getPower());
    }
}


