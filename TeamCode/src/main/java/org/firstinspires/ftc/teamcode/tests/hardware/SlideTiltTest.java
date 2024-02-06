package org.firstinspires.ftc.teamcode.tests.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Slide Tilt Test", group="tests")
@Config
public class SlideTiltTest extends OpMode {
    Servo tiltL, tiltR;

    DcMotor slideL, slideR;

    public static double servoPos = 0.5;
    public static int slidePos = 0;

    FtcDashboard dashboard;

    @Override
    public void init() {
        tiltL = hardwareMap.servo.get("tiltLeft");
        tiltR = hardwareMap.servo.get("tiltRight");

        tiltL.setDirection(Servo.Direction.REVERSE);

        tiltL.setPosition(servoPos);
        tiltR.setPosition(servoPos);

        slideL = hardwareMap.dcMotor.get("slidesLeft");
        slideR = hardwareMap.dcMotor.get("slidesRight");

        slideL.setPower(1.0);
        slideL.setTargetPosition(slidePos);
        slideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideR.setDirection(DcMotorSimple.Direction.REVERSE);
        slideR.setPower(1.0);
        slideR.setTargetPosition(slidePos);
        slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        double move = gamepad1.left_stick_y / 400;
        tiltL.setPosition(servoPos);
        tiltR.setPosition(servoPos);

        slideL.setTargetPosition(slidePos);
        slideR.setTargetPosition(slidePos);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("degrees", servoPos * 330.847 - 10.7458);

        dashboard.sendTelemetryPacket(packet);
    }
}
//E