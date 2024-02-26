package org.firstinspires.ftc.teamcode.tests.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Slide Tilt Test", group="tests")
@Config
public class SlideTiltTest extends OpMode {
    Servo tiltL, tiltR, depositTilt, deposit;

    DcMotor slideL, slideR;

    public static double slidesTilt = 0.5;
    public static int slidePos = 0;
    public static double depositTiltPos = 0.0;
    public static double depositPos = 0.0;

    FtcDashboard dashboard;

    @Override
    public void init() {
        tiltL = hardwareMap.servo.get("tiltLeft");
        tiltR = hardwareMap.servo.get("tiltRight");

        tiltL.setDirection(Servo.Direction.REVERSE);

        tiltL.setPosition(slidesTilt);
        tiltR.setPosition(slidesTilt);

        slideL = hardwareMap.dcMotor.get("slidesLeft");
        slideR = hardwareMap.dcMotor.get("slidesRight");

        slideL.setPower(1.0);
        slideL.setTargetPosition(slidePos);
        slideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideR.setDirection(DcMotorSimple.Direction.REVERSE);
        slideR.setPower(1.0);
        slideR.setTargetPosition(slidePos);
        slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        depositTilt = hardwareMap.servo.get("depositTilt");
        deposit = hardwareMap.servo.get("depositor");

        deposit.setPosition(depositPos);
        depositTilt.setPosition(depositTiltPos);

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        double move = gamepad1.left_stick_y / 400;
        tiltL.setPosition(slidesTilt);
        tiltR.setPosition(slidesTilt);

        slideL.setTargetPosition(slidePos);
        slideR.setTargetPosition(slidePos);

        depositTilt.setPosition(depositTiltPos);
        deposit.setPosition(depositPos);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("degrees", slidesTilt * 330.847 - 10.7458);

        dashboard.sendTelemetryPacket(packet);
    }
}
//E