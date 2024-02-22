package org.firstinspires.ftc.teamcode.tests.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;


@TeleOp(name="Deposit Test w Intake", group="tests")
public class DepositTest extends OpMode {
    Servo depositTilter;
    DcMotor roller;


    @Override
    public void init() {
        depositTilter = hardwareMap.servo.get("depositTilter");

        depositTilter.setPosition(ActuationConstants.Deposit.intakeTilt);
        roller = hardwareMap.dcMotor.get("intake");

        roller.setDirection(DcMotorSimple.Direction.REVERSE);
        roller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop() {
        depositTilter.setPosition(ActuationConstants.Deposit.depositTilt);

        if(gamepad1.left_trigger > 0.5)
            roller.setPower(1.0);
        else if(gamepad1.right_trigger > 0.5)
            roller.setPower(-1.0);
        else
            roller.setPower(0.0);

        roller.setPower(ActuationConstants.Intake.power);

    }

}
