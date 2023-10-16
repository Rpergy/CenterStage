package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Dragon Op")
public class DragonOp extends OpMode {
    DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void init() {
        frontLeft = hardwareMap.dcMotor.get("leftFront");
        frontRight = hardwareMap.dcMotor.get("rightFront");
        backLeft = hardwareMap.dcMotor.get("leftBack");
        backRight = hardwareMap.dcMotor.get("rightBack");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double move = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double movement = move + strafe + turn;

        frontLeft.setPower(movement);
        frontRight.setPower(movement);
        backLeft.setPower(movement);
        backRight.setPower(movement);
    }
}
