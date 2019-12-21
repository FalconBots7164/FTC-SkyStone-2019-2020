package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static android.os.SystemClock.sleep;

@TeleOp
public class teleOpModeDAM extends OpMode {
//damian
    // actual code for the main robot
    //xPushed was not used in Noah's previous code
    int xPushed = 0;

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor lift;
    DcMotor rightMotor;
    DcMotor leftMotor;
    DcMotor arm;
    Servo claw;
    Servo idol;
    CRServo armClaw;
    CRServo slide;
    CRServo sweeper;

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
//this kush is the code we took from mister Noah's Git
        rightMotor = hardwareMap.dcMotor.get("Right");
        leftMotor = hardwareMap.dcMotor.get("Left");
        lift = hardwareMap.dcMotor.get("Lift");
        arm = hardwareMap.dcMotor.get("Arm");
        claw = hardwareMap.servo.get("Claw");
        idol = hardwareMap.servo.get("Idol");
        armClaw = hardwareMap.crservo.get("ArmClaw");
        slide = hardwareMap.crservo.get("Slide");
        sweeper = hardwareMap.crservo.get("Sweeper");
        claw.setPosition(0);
        idol.setPosition(.9);
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
//            else {
//                telemetry.addData("ERR"," ERR");
//            }

        }

        telemetry.addData("Robot Version","v1.03");
        telemetry.addData("frontRight Power", frontRight.getPower());
        telemetry.addData("backRight Power", backRight.getPower());
        telemetry.addData("frontLeft Power", frontLeft.getPower());
        telemetry.addData("backLeft Power", backLeft.getPower());
    }
}

//Peoperoni W mayo
