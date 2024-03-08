package org.firstinspires.ftc.teamcode.tests.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Color Test", group = "tests")
public class ColorTest extends OpMode {
    ColorSensor top, bottom;

    FtcDashboard dashboard;

    @Override
    public void init() {
        top = hardwareMap.colorSensor.get("colorTop");
        bottom = hardwareMap.colorSensor.get("colorBottom");

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        telemetry.addData("left red", top.red());
        telemetry.addData("left green", top.green());
        telemetry.addData("left blue", top.blue());

        telemetry.addData("right red", bottom.red());
        telemetry.addData("right green", bottom.green());
        telemetry.addData("right blue", bottom.blue());

        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("top red", top.red());
        packet.put("top green", top.green());
        packet.put("top blue", top.blue());

        packet.put("bottom red", bottom.red());
        packet.put("bottom green", bottom.green());
        packet.put("bottom blue", bottom.blue());

        dashboard.sendTelemetryPacket(packet);

    }
}
