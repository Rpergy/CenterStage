package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;

import java.util.Arrays;


public class Actuation {

    public static boolean slowMode = false;
    public static boolean fieldCentric = false;
    public static boolean slides = false;
    private static boolean fieldCentricToggle = false;
    private static boolean slowModeToggle = false;
    private static boolean slidesToggle = false;

    public static DcMotor frontLeft, frontRight, backLeft, backRight;

    public static DcMotor slidesLeft, slidesRight;
    public static DcMotor intake;
    public static Servo tiltLeft, tiltRight;
    public static Servo depositTilter, deposit;
    public static Servo lifter;

    public static ModernRoboticsI2cRangeSensor rangeSensor;

    private static RevBlinkinLedDriver leds;

    private static double lastDist;
    private static double[] data;

    private static FtcDashboard dashboard;

    public static void setup(HardwareMap map, Telemetry telemetry) {
        AutoMovement.setup(map, telemetry);

        // nigerian prince scam

        if (map.dcMotor.contains("slidesLeft") && map.dcMotor.contains("slidesRight")) {
            slidesLeft = map.dcMotor.get("slidesLeft");
            slidesRight = map.dcMotor.get("slidesRight");

            slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            slidesLeft.setPower(1.0);
            slidesLeft.setTargetPosition(ActuationConstants.Extension.slidePositions[0]);
            slidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slidesRight.setDirection(DcMotorSimple.Direction.REVERSE);
            slidesRight.setPower(1.0);
            slidesRight.setTargetPosition(ActuationConstants.Extension.slidePositions[0]);
            slidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if(map.servo.contains("tiltLeft") && map.servo.contains("tiltRight")) {
            tiltLeft = map.servo.get("tiltLeft");
            tiltRight = map.servo.get("tiltRight");

            tiltLeft.setDirection(Servo.Direction.REVERSE);

            tiltLeft.setPosition(ActuationConstants.Extension.tiltPositions[0]);
            tiltRight.setPosition(ActuationConstants.Extension.tiltPositions[0]);
        }

        if (map.dcMotor.contains("intake")) {
            intake = map.dcMotor.get("intake");
        }

        if (map.servo.contains("depositTilter")) {
            depositTilter = map.servo.get("depositTilter");
        }

        if (map.servo.contains("deposit")) {
            deposit = map.servo.get("deposit");
        }

        if (map.servo.contains("lifter")) {
            lifter  = map.servo.get("lifter");
        }

        rangeSensor = map.get(ModernRoboticsI2cRangeSensor.class, "dist");

        if (!(Double.valueOf(rangeSensor.getDistance(DistanceUnit.INCH)).isNaN())) lastDist = rangeSensor.getDistance(DistanceUnit.INCH);

        data = new double[ActuationConstants.Extension.period];
        Arrays.fill(data, lastDist);

//        leds = map.get(RevBlinkinLedDriver.class, "lights");
//        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

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

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        dashboard = FtcDashboard.getInstance();
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

            frontLeft.setPower((newMove-turn-newStrafe) * multip);
            backLeft.setPower((newMove+turn-newStrafe) * multip);
            frontRight.setPower((newMove+turn+newStrafe) * multip);
            backRight.setPower((newMove-turn+newStrafe) * multip);
        }
        else {
            frontLeft.setPower((move-turn-strafe) * multip);
            backLeft.setPower((move+turn-strafe) * multip);
            frontRight.setPower((move+turn+strafe) * multip);
            backRight.setPower((move-turn+strafe) * multip);
        }

        slowModeToggle = toggleSlowMode;
        fieldCentricToggle = toggleFieldCentric;
    }

    public static void toggleSlides(boolean toggle) {
        double dist = lastDist;

        TelemetryPacket packet = new TelemetryPacket();

        double newDist = rangeSensor.getDistance(DistanceUnit.INCH);

        if(!Double.isNaN(newDist) && Math.abs(lastDist - newDist) < 30)
            dist = newDist;

        if (data.length - 1 >= 0) System.arraycopy(data, 0, data, 1, data.length - 1);
        data[0] = dist;

        double smoothDist = 0;
        for(double val : data) smoothDist += val;
        smoothDist /= ActuationConstants.Extension.period;

        packet.put("dist", smoothDist);
        dashboard.sendTelemetryPacket(packet);

        lastDist = smoothDist;

        if (toggle && !slidesToggle) {
            slides = !slides;

            if (!slides) {
                tiltLeft.setPosition(ActuationConstants.Extension.tiltPositions[0]);
                tiltRight.setPosition(ActuationConstants.Extension.tiltPositions[0]);

                slidesLeft.setTargetPosition(0);
                slidesRight.setTargetPosition(0);
            }
        }

        if(slides) {
            tiltLeft.setPosition(ActuationConstants.Extension.tiltPositions[1]);
            tiltRight.setPosition(ActuationConstants.Extension.tiltPositions[1]);

            int slidePos = (int)(smoothDist * 133.869 + 434.94);

            if(slidePos <= ActuationConstants.Extension.maxExtend) {
                slidesLeft.setTargetPosition(slidePos);
                slidesRight.setTargetPosition(slidePos);
            }
            else {
                slidesLeft.setTargetPosition(ActuationConstants.Extension.maxExtend);
                slidesRight.setTargetPosition(ActuationConstants.Extension.maxExtend);
            }
        }

        slidesToggle = toggle;
    }

    public static void slidesOut() {
        double dist = lastDist;

        double newDist = rangeSensor.getDistance(DistanceUnit.INCH);

        if(!Double.isNaN(newDist))
            dist = newDist;

        tiltLeft.setPosition(ActuationConstants.Extension.tiltPositions[1]);
        tiltRight.setPosition(ActuationConstants.Extension.tiltPositions[1]);

        int slidePos = (int)(dist * 133.869 + 434.94);

        if(slidePos <= ActuationConstants.Extension.maxExtend) {
            slidesLeft.setTargetPosition(slidePos);
            slidesRight.setTargetPosition(slidePos);
        }
        else {
            slidesLeft.setTargetPosition(ActuationConstants.Extension.maxExtend);
            slidesRight.setTargetPosition(ActuationConstants.Extension.maxExtend);
        }
    }

    public static void slidesIn() {
        tiltLeft.setPosition(ActuationConstants.Extension.tiltPositions[0]);
        tiltRight.setPosition(ActuationConstants.Extension.tiltPositions[0]);

        slidesLeft.setTargetPosition(0);
        slidesRight.setTargetPosition(0);
    }

    public static void setIntake(double power) {
        intake.setPower(power);
    }

    public static void setIntakeArm(double pos) {  }

    public static void setTilt(double pos) {
        tiltLeft.setPosition(pos);
        tiltRight.setPosition(pos);
    }

    public static void setSlides(int pos) {
        slidesLeft.setTargetPosition(pos);
        slidesRight.setTargetPosition(pos);
    }

    public static void initDepositTilter(boolean toggle){
        if(toggle){
            depositTilter.setPosition(ActuationConstants.Deposit.depositTilt);
        }
    }
}
