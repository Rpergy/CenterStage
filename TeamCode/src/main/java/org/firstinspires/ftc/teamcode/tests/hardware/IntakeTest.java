package org.firstinspires.ftc.teamcode.tests.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Intake Test", group="tests")
public class IntakeTest extends OpMode {
    DcMotor roller;

    @Override
    public void init() {
        roller = hardwareMap.dcMotor.get("intake");
    }

    @Override
    public void loop() {
        if(gamepad1.left_trigger > 0.5)
            roller.setPower(1.0);
        else if(gamepad1.right_trigger > 0.5)
            roller.setPower(-1.0);
        else
            roller.setPower(0.0);
    }
}
