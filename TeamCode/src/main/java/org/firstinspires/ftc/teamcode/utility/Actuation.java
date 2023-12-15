package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Actuation {
    private static boolean closeClawL = false;
    private static boolean closeClawR = false;

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
            arm.setPosition(0.3);
        }

        if (map.servo.contains("clawWrist")) {
            wrist = map.servo.get("clawWrist");
            wrist.setPosition(ActuationConstants.Claw.wristAutoInit);
        }

        if (map.servo.contains("leftClaw")) {
            lClaw = map.servo.get("leftClaw");
            lClaw.setPosition(ActuationConstants.Claw.closed);
        }

        if (map.servo.contains("rightClaw")) {
            rClaw = map.servo.get("rightClaw");
            rClaw.setDirection(Servo.Direction.REVERSE);
            rClaw.setPosition(ActuationConstants.Claw.closed);
        }

        if (map.dcMotor.contains("extension")) {
            extension = map.dcMotor.get("extension");
            extension.setPower(1.0);
            extension.setTargetPosition(ActuationConstants.Extension.extensionStart);
            extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        frontLeft  = map.get(DcMotor.class, "frontLeft");
        frontRight = map.get(DcMotor.class, "frontRight");
        backLeft = map.get(DcMotor.class, "backLeft");
        backRight = map.get(DcMotor.class, "backRight");

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
            closeClawL = !closeClawL;
            closeClawR = !closeClawR;
            lClaw.setPosition(closeClawL ? ActuationConstants.Claw.closed : ActuationConstants.Claw.open);
            rClaw.setPosition(closeClawL ? ActuationConstants.Claw.closed : ActuationConstants.Claw.open);
            clawToggle = true;
        }
        else if (!input) {
            clawToggle = false;
        }
    }

    public static boolean getClawState() {
        return closeClawL || closeClawR;
    }

    public static void toggleLClaw(boolean input) {
        if (input && !clawToggle) {
            closeClawL = !closeClawL;
            lClaw.setPosition(closeClawL ? ActuationConstants.Claw.closed : ActuationConstants.Claw.open);
            clawToggle = true;
        }
        else if (!input) {
            clawToggle = false;
        }
    }

    public static void toggleRClaw(boolean input) {
        if (input && !clawToggle) {
            closeClawR = !closeClawR;
            rClaw.setPosition(closeClawR ? ActuationConstants.Claw.closed : ActuationConstants.Claw.open);
            clawToggle = true;
        }
        else if (!input) {
            clawToggle = false;
        }
    }

    public static void setClaw(double input) {
        lClaw.setPosition(input);
        rClaw.setPosition(input);
    }

    public static void setLClaw(double input) {
        lClaw.setPosition(input);
    }

    public static void setRClaw(double input) {
        rClaw.setPosition(input);
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

    public static void setWrist(double input) {
        wrist.setPosition(input);
    }

    public static void incrementExtension(int posChange) {
//        if (posChange + extension.getCurrentPosition() > ActuationConstants.Extension.maxExtension) posChange = ActuationConstants.Extension.maxExtension - extension.getCurrentPosition();
        extension.setTargetPosition(extension.getCurrentPosition() + posChange);
    }

    public static void setExtension(int newPos) {
        if (newPos > ActuationConstants.Extension.maxExtension) newPos = ActuationConstants.Extension.maxExtension;
        extension.setTargetPosition(-newPos);
    }

    public static int getExtension() {
        return extension.getCurrentPosition();
    }

    public static void setTilt(double input) {
        arm.setPosition(input);
    }
    public static double getTilt() { return arm.getPosition(); }
}
