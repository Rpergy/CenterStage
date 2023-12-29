package org.firstinspires.ftc.teamcode.utility.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Intake Test")
public class IntakeTest extends OpMode {
    DcMotor intake;
    @Override
    public void init() {
        intake = hardwareMap.dcMotor.get("intake");
    }

    @Override
    public void loop() {
        intake.setPower(gamepad1.left_stick_y);
    }
}
