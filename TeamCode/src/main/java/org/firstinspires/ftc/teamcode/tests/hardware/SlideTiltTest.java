package org.firstinspires.ftc.teamcode.tests.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.Actuation;

@TeleOp(name="Slide Tilt Test", group="tests")
@Config
public class SlideTiltTest extends OpMode {
    public static double slidesTilt = 0.5;
    public static int slidePos = 0;
    public static double depositTiltPos = 0.0;
    public static double depositSpeed = 0.0;

    FtcDashboard dashboard;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        double move = gamepad1.left_stick_y / 400;

        Actuation.setDeposit(depositSpeed);
        Actuation.setDepositTilt(depositTiltPos);
        Actuation.setTilt(slidesTilt);
        Actuation.setSlides(slidePos);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("degrees", slidesTilt * 330.847 - 10.7458);

        dashboard.sendTelemetryPacket(packet);
    }
}
//E