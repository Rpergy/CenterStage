package org.firstinspires.ftc.teamcode.tests;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="april tag slides", group = "tests")
@Config
public class AprilTagSlides extends OpMode {
    ModernRoboticsI2cRangeSensor rangeSensor;

    Servo tiltL, tiltR;

    DcMotor slideL, slideR;

    public static double servoPos = 0.5;
    public static int slidePos = 0;

    double lastDist = 0;

    FtcDashboard dashboard;

    @Override
    public void init() {
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "dist");

        if (!(Double.valueOf(rangeSensor.getDistance(DistanceUnit.INCH)).isNaN())) lastDist = rangeSensor.getDistance(DistanceUnit.INCH);

        tiltL = hardwareMap.servo.get("tiltLeft");
        tiltR = hardwareMap.servo.get("tiltRight");

        tiltL.setDirection(Servo.Direction.REVERSE);

        tiltL.setPosition(servoPos);
        tiltR.setPosition(servoPos);

        slideL = hardwareMap.dcMotor.get("slidesLeft");
        slideR = hardwareMap.dcMotor.get("slidesRight");

        slideL.setPower(1.0);
        slideL.setTargetPosition(slidePos);
        slideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideR.setDirection(DcMotorSimple.Direction.REVERSE);
        slideR.setPower(1.0);
        slideR.setTargetPosition(slidePos);
        slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("dist", lastDist);
        packet.put("length", slidePos);

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void loop() { // for i in range(i+1),
        double dist = lastDist;
        if(Math.abs(lastDist - rangeSensor.getDistance(DistanceUnit.INCH)) < 5 && !Double.valueOf(rangeSensor.getDistance(DistanceUnit.INCH)).isNaN())
            dist = rangeSensor.getDistance(DistanceUnit.INCH);

        tiltL.setPosition(servoPos);
        tiltR.setPosition(servoPos);

        slidePos = (int)(dist * 133.869 + 734.94) - 300;

        if(slidePos <= 2600) {
            slideL.setTargetPosition(slidePos);
            slideR.setTargetPosition(slidePos);
        }
        else {
            slideL.setTargetPosition(2600);
            slideR.setTargetPosition(2600);
        }

        telemetry.addData("Distance", rangeSensor.getDistance(DistanceUnit.INCH));

        lastDist = dist;

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("dist", dist);
        packet.put("slidePos", slidePos);

        dashboard.sendTelemetryPacket(packet);
    }
}
