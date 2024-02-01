package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    public static boolean slowMode = false;
    public static boolean fieldCentric = false;
    private static boolean fieldCentricToggle = false;
    private static boolean slowModeToggle = false;

    public static DcMotor frontLeft, frontRight, backLeft, backRight;

    public static DcMotor slidesLeft, slidesRight;
    public static Servo tiltLeft, tiltRight;
    public static Servo depositTilt, deposit;

    private static RevBlinkinLedDriver leds;

    public static void setup(HardwareMap map, Telemetry telemetry) {
        AutoMovement.setup(map, telemetry);

        // nigerian prince scam

        if (map.dcMotor.contains("slidesLeft") && map.dcMotor.contains("slidesRight")) {
            slidesLeft = map.dcMotor.get("slidesLeft");
            slidesRight = map.dcMotor.get("slidesRight");

            slidesLeft.setPower(1.0);
            slidesLeft.setTargetPosition(ActuationConstants.Extension.slidePositions[0]);
            slidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slidesRight.setDirection(DcMotorSimple.Direction.REVERSE);
            slidesRight.setPower(1.0);
            slidesRight.setTargetPosition(ActuationConstants.Extension.slidePositions[0]);
            slidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if(map.servo.contains("tiltLeft") && map.servo.contains("tiltRight")) {
            tiltLeft.setDirection(Servo.Direction.REVERSE);

            tiltLeft.setPosition(ActuationConstants.Extension.tiltPositions[0]);
            tiltRight.setPosition(ActuationConstants.Extension.tiltPositions[0]);
        }

        if (map.servo.contains("depositTilt")) {
            depositTilt = map.servo.get("depositTilt");
        }

        if (map.servo.contains("deposit")) {
            deposit = map.servo.get("deposit");
        }

        leds = map.get(RevBlinkinLedDriver.class, "lights");
        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

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

    public static void setTilt(double pos) {
        tiltLeft.setPosition(pos);
        tiltRight.setPosition(pos);
    }

    public static void setSlides(int pos) {
        slidesLeft.setTargetPosition(pos);
        slidesRight.setTargetPosition(pos);
    }
}
