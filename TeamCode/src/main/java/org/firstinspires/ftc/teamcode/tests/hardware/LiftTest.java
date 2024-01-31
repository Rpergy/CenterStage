package org.firstinspires.ftc.teamcode.tests.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;

import java.nio.file.attribute.AclEntryType;

@TeleOp(name = "Lift Test", group="tests")
@Config
public class LiftTest extends OpMode {
    public static double tiltPos, clawPos, wristPos;

    public static int extensionPos;

    @Override
    public void init() {
        extensionPos = ActuationConstants.Extension.extensionStart;
        tiltPos = ActuationConstants.Extension.tiltIntake;
        clawPos = ActuationConstants.Claw.open;
        wristPos = ActuationConstants.Claw.wristIntake;

        Actuation.setup(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        Actuation.setExtension(extensionPos);
        Actuation.setClaw(clawPos);
        Actuation.setWrist(wristPos);
        Actuation.setTilt(tiltPos);
    }
}