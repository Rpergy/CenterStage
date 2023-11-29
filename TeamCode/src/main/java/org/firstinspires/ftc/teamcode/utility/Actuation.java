package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Actuation {
    private static boolean closeClaw = false;
    private static boolean wristUp = false;
    private static boolean tiltUp = false;

    private static Servo arm, wrist, lClaw, rClaw;
    private static DcMotor extension;

    private static boolean clawToggle = false;
    private static boolean wristToggle = false;
    private static boolean tiltToggle = false;

    private static DcMotor frontLeft, frontRight, backLeft, backRight;

    public static void setup(HardwareMap map) {
        if (map.servo.contains("extensionTilt")) {
            arm = map.servo.get("extensionTilt");
            arm.setPosition(ActuationConstants.Extension.tiltIntake);
        }

        if (map.servo.contains("clawWrist")) {
            wrist = map.servo.get("clawWrist");
            wrist.setPosition(ActuationConstants.Claw.wristIntake);
        }

        if (map.servo.contains("leftClaw")) {
            lClaw = map.servo.get("leftClaw");
            lClaw.setPosition(ActuationConstants.Claw.open);
        }

        if (map.servo.contains("rightClaw")) {
            rClaw = map.servo.get("rightClaw");
            rClaw.setDirection(Servo.Direction.REVERSE);
            rClaw.setPosition(ActuationConstants.Claw.open);
        }

        if (map.dcMotor.contains("extension")) {
            extension = map.dcMotor.get("extension");
            extension.setPower(1.0);
            extension.setTargetPosition(ActuationConstants.Extension.extensionStart);
            extension.setDirection(DcMotorSimple.Direction.REVERSE);
            extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        frontLeft  = map.get(DcMotor.class, "frontLeft");
        frontRight = map.get(DcMotor.class, "frontRight");
        backLeft = map.get(DcMotor.class, "backLeft");
        backRight = map.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    public static void drive(double move, double turn, double strafe) {
        frontLeft.setPower(move - turn - strafe);
        frontRight.setPower(move + turn + strafe);
        backLeft.setPower(move - turn + strafe);
        backRight.setPower(move + turn - strafe);
    }

    public static void toggleClaw(boolean input) {
        if (input && !clawToggle) {
            closeClaw = !closeClaw;
            lClaw.setPosition(closeClaw ? ActuationConstants.Claw.closed : ActuationConstants.Claw.open);
            rClaw.setPosition(closeClaw ? ActuationConstants.Claw.closed : ActuationConstants.Claw.open);
            clawToggle = true;
        }
        else if (!input) {
            clawToggle = false;
        }
    }

    public static void toggleWrist(boolean input) {
        if (input && !wristToggle) {
            wristUp = !wristUp;
            wrist.setPosition(wristUp ? ActuationConstants.Claw.wristDeposit : ActuationConstants.Claw.wristIntake);
            wristToggle = true;
        }
        else if (!input) {
            wristToggle = false;
        }
    }

    public static void setExtension(int posChange) {
        extension.setTargetPosition(extension.getCurrentPosition() + posChange);
    }

    public static void toggleTilt(boolean input) {
        if (input && !tiltToggle) {
            tiltUp = !tiltUp;
            arm.setPosition(tiltUp ? ActuationConstants.Extension.tiltDeposit : ActuationConstants.Extension.tiltIntake);
            tiltToggle = true;
        }
        else if (!input) {
            tiltToggle = false;
        }
    }
}
