package org.firstinspires.ftc.teamcode.tests.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Slide Tilt Test", group="tests")
public class SlideTiltTest extends OpMode {
    Servo tiltL, tiltR;
    @Override
    public void init() {
        tiltL = hardwareMap.servo.get("tiltL");
        tiltR = hardwareMap.servo.get("tiltR");

        tiltL.setPosition(1.0);
        tiltR.setPosition(1.0);
    }

    @Override
    public void loop() {
        double move = gamepad1.left_stick_y / 400;
        tiltL.setPosition(tiltL.getPosition() + move);
        tiltR.setPosition(tiltR.getPosition() + move);
    }
}
