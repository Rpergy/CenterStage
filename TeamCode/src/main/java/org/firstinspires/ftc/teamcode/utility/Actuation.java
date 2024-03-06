package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.Arrays;


public class Actuation {

    public static boolean slowMode = false;
    public static boolean fieldCentric = false;
    public static boolean slides = false;
    public static boolean tilt = false;
    public static int tiltPos = 0;
    public static int stuckStatus = 0;
    public static boolean deposit = false;
    private static boolean fieldCentricToggle = false;
    private static boolean slowModeToggle = false;
    private static boolean slidesToggle = false;
    private static boolean tiltToggle = false;
    private static boolean depositToggle = false;
    private static boolean stuckToggle = false;

    public static DcMotor frontLeft, frontRight, backLeft, backRight;

    public static DcMotor slidesLeft, slidesRight;
    public static DcMotor intake;
    public static Servo tiltLeft, tiltRight;
    public static Servo depositTilt, depositor;
    public static Servo lifter;
    public static Servo airplaneTilt, airplaneLaunch;

    public static ModernRoboticsI2cRangeSensor rangeSensor;

    private static RevBlinkinLedDriver leds;

    private static double lastDist = 0;
    private static double[] data;

    private static FtcDashboard dashboard;

    private static Telemetry telemetry;

    public static void setup(HardwareMap map, Telemetry tel) {
        AutoMovement.setup(map, telemetry);

        telemetry = tel;

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

        if (map.servo.contains("depositTilt")) {
            depositTilt = map.servo.get("depositTilt");
            depositTilt.setPosition(ActuationConstants.Deposit.intakeTilt);
        }

        if (map.servo.contains("depositor")) {
            depositor = map.servo.get("depositor");
        }

        if (map.servo.contains("lifter")) {
            lifter = map.servo.get("lifter");
            lifter.setPosition(ActuationConstants.Intake.stackPos[0]);
        }

        if (map.servo.contains("airplaneLaunch")) {
            airplaneLaunch = map.servo.get("airplaneLaunch");
            airplaneLaunch.setPosition(ActuationConstants.Plane.releaseDown);
        }

        if (map.servo.contains("airplaneTilt")) {
            airplaneTilt = map.servo.get("airplaneTilt");
            airplaneTilt.setPosition(ActuationConstants.Plane.tilt);
        }

        rangeSensor = map.get(ModernRoboticsI2cRangeSensor.class, "dist");

        data = new double[ActuationConstants.Extension.period];
        Arrays.fill(data, -1);

        leds = map.get(RevBlinkinLedDriver.class, "leds");
        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

        frontLeft  = map.get(DcMotor.class, "frontLeft");
        frontRight = map.get(DcMotor.class, "frontRight");
        backLeft = map.get(DcMotor.class, "backLeft");
        backRight = map.get(DcMotor.class, "backRight");

//        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("dist", 0);
        packet.put("smoothDist", 0);
        packet.put("travel dist", 0);
        packet.put("motor power", 0);
        dashboard.sendTelemetryPacket(packet);
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

    public static double getDist() {
        double dist = lastDist;

        TelemetryPacket packet = new TelemetryPacket();

        double newDist = rangeSensor.getDistance(DistanceUnit.INCH);

        if(!Double.isNaN(newDist) && Math.abs(lastDist - newDist) < 30 && newDist != Double.POSITIVE_INFINITY && newDist != Double.NEGATIVE_INFINITY)
            dist = newDist;

        if (data.length - 1 >= 0) System.arraycopy(data, 0, data, 1, data.length - 1);
        data[0] = dist;

        double smoothDist = 0;
        double validPoints = 0;
        for(double val : data)
            if(val != -1) {
                smoothDist += val;
                validPoints += 1;
            }
        smoothDist /= validPoints;

        packet.put("dist", dist);
        packet.put("smoothDist", smoothDist);
        dashboard.sendTelemetryPacket(packet);

        lastDist = smoothDist;

        return smoothDist;
    }

    public static void canvasAlign() {
        double travelDist = getDist() - 6;

        TelemetryPacket packet = new TelemetryPacket();

        while(Math.abs(travelDist) > 1.0) {
            drive(-Math.tanh(travelDist/5) * 0.3, 0.0, 0.0);
            travelDist = getDist() - 6;

            packet.put("travel dist", travelDist);
            packet.put("motor power", -Math.tanh(travelDist/5) * 0.3);
            dashboard.sendTelemetryPacket(packet);
        }
        drive(0, 0, 0);
        AutoMovement.robotPose = new Pose(48, AutoMovement.robotPose.y, AutoMovement.robotPose.heading);
    }

    public static void toggleSlides(boolean toggle) {
        double dist = getDist();

        if (toggle && !slidesToggle && tiltPos != 0) {
            slides = !slides;

            if (!slides) {
                slidesLeft.setTargetPosition(0);
                slidesRight.setTargetPosition(0);
            }
        }

        if(slides) {
            int slidePos = 0;
            if (tiltPos == 1)
                slidePos = (int)(dist * 125 + 400);
            else if (tiltPos == 2)
                slidePos = (int)(dist * 197 + 775);
            else if (tiltPos == 3)
                slidePos = 2400;

            if(slidePos <= ActuationConstants.Extension.maxExtend) {
                slidesLeft.setTargetPosition(slidePos);
                slidesRight.setTargetPosition(slidePos);
            }
            else {
                slidesLeft.setTargetPosition(ActuationConstants.Extension.maxExtend);
                slidesRight.setTargetPosition(ActuationConstants.Extension.maxExtend);
            }
        }

        telemetry.addData("tiltPos", tiltPos);
        telemetry.addData("slides", slides);

        slidesToggle = toggle;
    }

    public static void toggleTilt(boolean toggle) {
        if (toggle && !tiltToggle && !slides) {
            if(tiltPos == 0 || tiltPos == 1) tiltPos += 1;
            else tiltPos = 0;

            deposit = (tiltPos >= 1);
        }
        tiltLeft.setPosition(ActuationConstants.Extension.tiltPositions[tiltPos]);
        tiltRight.setPosition(ActuationConstants.Extension.tiltPositions[tiltPos]);

        tiltToggle = toggle;
    }

    public static void setTiltPreset(int num) {
        if(!slides)
            tiltPos = num;

        tiltLeft.setPosition(ActuationConstants.Extension.tiltPositions[tiltPos]);
        tiltRight.setPosition(ActuationConstants.Extension.tiltPositions[tiltPos]);

        if(num != 0) {
            deposit = true;
        }
        else {
            deposit = false;
        }
    }

    public static void slidesOut() {
        double dist = lastDist;

        double newDist = rangeSensor.getDistance(DistanceUnit.INCH);

        if(!Double.isNaN(newDist))
            dist = newDist;

        int slidePos = (int)(dist * 125 + 400);

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
        slidesLeft.setTargetPosition(0);
        slidesRight.setTargetPosition(0);
    }

    public static void setIntake(double power) {
        intake.setPower(power);
    }

    public static void setIntakeArm(double pos) { lifter.setPosition(pos); }

    public static void setTilt(double pos) {
        tiltLeft.setPosition(pos);
        tiltRight.setPosition(pos);
    }

    public static void setSlides(int pos) {
        slidesLeft.setTargetPosition(pos);
        slidesRight.setTargetPosition(pos);
    }

    public static void autoDeposit() {
        int slideAvg = (slidesLeft.getCurrentPosition() + slidesRight.getCurrentPosition()) / 2;

        if (slideAvg > 900 && tiltPos > 0) {
            depositTilt.setPosition(ActuationConstants.Deposit.depositTilts[tiltPos-1]);
        }
        else {
            depositTilt.setPosition(ActuationConstants.Deposit.intakeTilt);
        }
    }

    public static void setDepositTilt(double pos) {
        depositTilt.setPosition(pos);
    }

    public static void setDeposit(double pos) { depositor.setPosition(pos); }

    public static void toggleDeposit(boolean toggle) {
        telemetry.addData("toggle", toggle);
        telemetry.addData("lastToggle", depositToggle);
        telemetry.addData("depositing", deposit);

        if(toggle && !depositToggle) {
            deposit = !deposit;
        }

        if (!deposit) {
            depositor.setPosition(ActuationConstants.Deposit.open);
        }
        else {
            depositor.setPosition(ActuationConstants.Deposit.closed);
        }

        depositToggle = toggle;
    }

    public static void stuckFix(boolean toggle) {
        if(toggle && !stuckToggle) {
            if(stuckStatus != 3)
                stuckStatus += 1;
            else
                stuckStatus = 0;

            if(stuckStatus == 0) setSlides(0);
            else if(stuckStatus == 1) setTiltPreset(1);
            else if(stuckStatus == 2) setSlides(500);
            else if(stuckStatus == 3) setTiltPreset(0);
        }
        telemetry.addData("stuck status", stuckStatus);

        stuckToggle = toggle;
    }

    public static void hangSetup() {
        setTilt(ActuationConstants.Extension.tiltPositions[2]);
        setSlides(ActuationConstants.Extension.hang);
    }

    public static void hang() {
        setSlides(0);
    }

    public static void setLeds(RevBlinkinLedDriver.BlinkinPattern pattern) {
        leds.setPattern(pattern);
    }
}
