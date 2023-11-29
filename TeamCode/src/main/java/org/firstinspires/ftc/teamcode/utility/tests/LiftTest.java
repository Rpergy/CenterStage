package org.firstinspires.ftc.teamcode.utility.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;

import java.nio.file.attribute.AclEntryType;

@TeleOp(name = "Lift Test")
@Config
public class LiftTest extends OpMode {
    DcMotor extension;
    Servo tilt, wrist, leftClaw, rightClaw;

    public static double tiltPos, clawPos, wristPos;

    public static int extensionPos;

    @Override
    public void init() {
        extensionPos = ActuationConstants.Extension.extensionStart;
        tiltPos = ActuationConstants.Extension.tiltIntake;
        clawPos = ActuationConstants.Claw.open;
        wristPos = ActuationConstants.Claw.wristIntake;

        extension = hardwareMap.dcMotor.get("extension");
        extension.setPower(1.0);
        extension.setTargetPosition(ActuationConstants.Extension.extensionStart);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        tilt = hardwareMap.servo.get("extensionTilt");
        tilt.setPosition(ActuationConstants.Extension.tiltIntake);

        wrist = hardwareMap.servo.get("clawWrist");
        wrist.setPosition(ActuationConstants.Claw.wristIntake);

        leftClaw = hardwareMap.servo.get("leftClaw");
        leftClaw.setPosition(ActuationConstants.Claw.open);

        rightClaw = hardwareMap.servo.get("rightClaw");
        rightClaw.setDirection(Servo.Direction.REVERSE);
        rightClaw.setPosition(ActuationConstants.Claw.open);
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