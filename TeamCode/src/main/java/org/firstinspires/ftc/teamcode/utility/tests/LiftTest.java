package org.firstinspires.ftc.teamcode.utility.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.ActuationConstants;

@TeleOp(name = "Lift Test")
@Config
public class LiftTest extends OpMode {
    DcMotor extension;
    Servo tilt, wrist, leftClaw, rightClaw;

    double tiltPos, clawPos, wristPos;
    int extensionPos;

    @Override
    public void init() {
        extensionPos = 0;
        tiltPos = 0.0;
        clawPos = 0.0;
        wristPos = 0.0;

        extension = hardwareMap.dcMotor.get("extension");
        extension.setPower(1.0);
        extension.setTargetPosition(extensionPos);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        tilt = hardwareMap.servo.get("extensionTilt");
        tilt.setPosition(tiltPos);

        wrist = hardwareMap.servo.get("clawWrist");
        wrist.setPosition(wristPos);

        leftClaw = hardwareMap.servo.get("leftClaw");
        leftClaw.setPosition(clawPos);

        rightClaw = hardwareMap.servo.get("rightClaw");
        rightClaw.setDirection(Servo.Direction.REVERSE);
        rightClaw.setPosition(clawPos);
    }

    @Override
    public void loop() {
        extension.setTargetPosition(extensionPos);
        tilt.setPosition(tiltPos);

        wrist.setPosition(wristPos);
        leftClaw.setPosition(clawPos);
        rightClaw.setPosition(clawPos);
    }
}
