package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Wheel Test", group="tests")
@Disabled
public class WheelTest extends OpMode {
    DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void init() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (gamepad1.triangle) frontRight.setPower(0.5);
        else frontRight.setPower(0.0);
        if (gamepad1.square) frontLeft.setPower(0.5);
        else frontLeft.setPower(0.0);
        if (gamepad1.cross) backLeft.setPower(0.5);
        else backLeft.setPower(0.0);
        if (gamepad1.circle) backRight.setPower(0.5);
        else backRight.setPower(0.0);
    }
}
