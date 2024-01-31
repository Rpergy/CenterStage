package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.dataTypes.PixelColors;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;


public class Actuation {
    private static boolean closeClawL = false;
    private static boolean closeClawR = false;

    private static boolean wristUp = false;

    public static boolean slowMode = false;
    public static boolean fieldCentric = false;

    private static boolean clawToggle = false;
    private static boolean wristToggle = false;
    private static boolean fieldCentricToggle = false;
    private static boolean slowModeToggle = false;

    static Servo arm, wrist, lClaw, rClaw;
    static DcMotor extension;

    public static DcMotor frontLeft, frontRight, backLeft, backRight;

    private static RevBlinkinLedDriver leds;

    private static ColorSensor colorLeft, colorRight;

    private static IMU imu;

    public static void setup(HardwareMap map, Telemetry telemetry) {
        AutoMovement.setup(map, telemetry);

        // nigerian prince scam

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

        leds = map.get(RevBlinkinLedDriver.class, "lights");
        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

        if (map.colorSensor.contains("colorLeft")) colorLeft = map.colorSensor.get("colorLeft");
        if (map.colorSensor.contains("colorRight")) colorRight = map.colorSensor.get("colorRight");

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

        imu = map.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        )));

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    public static void drive(double move, double turn, double strafe) {
        frontLeft.setPower(move - turn - strafe);
        frontRight.setPower(move + turn + strafe);
        backLeft.setPower(move - turn + strafe);
        backRight.setPower(move + turn - strafe);
    }

    public static void teleDrive(boolean toggleSlowMode, boolean toggleFieldCentric, double move, double turn, double strafe) {
        if (toggleSlowMode && !slowModeToggle) slowMode = !slowMode;
        if (toggleFieldCentric && !fieldCentricToggle) fieldCentric = !fieldCentric;

        double multip = (slowMode) ? 0.5 : 1.0;
        if (fieldCentric) {
            double newMove = strafe*Math.sin(-AutoMovement.robotPose.heading)+move*Math.cos(-AutoMovement.robotPose.heading);
            double newStrafe = strafe*Math.cos(-AutoMovement.robotPose.heading)-move*Math.sin(-AutoMovement.robotPose.heading);

            frontLeft.setPower((newMove+newStrafe+turn) * multip);
            backLeft.setPower((newMove-newStrafe+turn) * multip);
            frontRight.setPower((newMove-newStrafe-turn) * multip);
            backRight.setPower((newMove+newStrafe-turn) * multip);
        }
        else {
            frontLeft.setPower((move+strafe+turn) * multip);
            backLeft.setPower((move-strafe+turn) * multip);
            frontRight.setPower((move-strafe-turn) * multip);
            backRight.setPower((move+strafe-turn) * multip);
        }

        slowModeToggle = toggleSlowMode;
        fieldCentricToggle = toggleFieldCentric;
    }
    public static PixelColors leftColors() {
        if (colorLeft.red() < 500 && colorLeft.green() < 500 && colorLeft.blue() < 500)
            return PixelColors.EMPTY;
        else if (colorLeft.green() > colorLeft.blue() && colorLeft.green() > colorLeft.red())
            return PixelColors.GREEN;
        else if (colorLeft.green() > colorLeft.blue() && colorLeft.red() > colorLeft.blue())
            return PixelColors.YELLOW;
        else if (colorLeft.blue() > colorLeft.red() && colorLeft.green() > colorLeft.red())
            return PixelColors.PURPLE;
        else
            return PixelColors.WHITE;
    }

    public static PixelColors rightColors() {
        if (colorRight.red() < 500 && colorRight.green() < 500 && colorRight.blue() < 500)
            return PixelColors.EMPTY;
        else if (colorRight.green() > colorRight.blue() && colorRight.green() > colorRight.red())
            return PixelColors.GREEN;
        else if (colorRight.green() > colorRight.blue() && colorRight.red() > colorRight.blue())
            return PixelColors.YELLOW;
        else if (colorRight.blue() > colorRight.red() && colorRight.green() > colorRight.red())
            return PixelColors.PURPLE;
        else
            return PixelColors.WHITE;
    }

    public static void setColors() {
        if (rightColors() == PixelColors.EMPTY && leftColors() == PixelColors.EMPTY)
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        else if (rightColors() != PixelColors.EMPTY && leftColors() == PixelColors.EMPTY)
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        else if (rightColors() == PixelColors.EMPTY && leftColors() != PixelColors.EMPTY)
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        else
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
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
