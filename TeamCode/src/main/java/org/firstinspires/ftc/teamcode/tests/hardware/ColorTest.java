package org.firstinspires.ftc.teamcode.tests.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Color Test", group = "tests")
@Disabled
public class ColorTest extends OpMode {
    ColorSensor left, right;

    @Override
    public void init() {
        left = hardwareMap.colorSensor.get("colorLeft");
        right = hardwareMap.colorSensor.get("colorRight");
    }

    @Override
    public void loop() {
        telemetry.addData("left red", left.red());
        telemetry.addData("left green", left.green());
        telemetry.addData("left blue", left.blue());

        telemetry.addData("right red", right.red());
        telemetry.addData("right green", right.green());
        telemetry.addData("right blue", right.blue());

        telemetry.update();

    }
}
