package org.firstinspires.ftc.teamcode.tests.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Intake Test", group="tests")
public class IntakeTest extends OpMode {
    DcMotor roller;

    @Override
    public void init() {
        roller = hardwareMap.dcMotor.get("roller");
    }

    @Override
    public void loop() {
        roller.setPower(gamepad1.left_stick_y);
    }
}
