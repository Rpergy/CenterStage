package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

@TeleOp(name="distance slides", group = "tests")
@Config
public class DistanceSlides extends OpMode {
    ModernRoboticsI2cRangeSensor rangeSensor;
    Servo tiltL, tiltR;
    DcMotor slideL, slideR;

    public static double servoPos = 0.5;
    public static int slidePos = 0;

    double lastDist = 0;

    int period = 20;

    double[] data;

    FtcDashboard dashboard;

    @Override
    public void init() {
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "dist");

        if (!(Double.valueOf(rangeSensor.getDistance(DistanceUnit.INCH)).isNaN())) lastDist = rangeSensor.getDistance(DistanceUnit.INCH);

        data = new double[period];
        Arrays.fill(data, lastDist);

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
        packet.put("smoothDist", lastDist);
        packet.put("length", slidePos);

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void loop() {
        double dist = lastDist;

        double newDist = rangeSensor.getDistance(DistanceUnit.INCH);

        if(!Double.isNaN(newDist) && Math.abs(lastDist - newDist) < 3)
            dist = newDist;

        for(int i = data.length-1; i > 0; i--) {
            data[i] = data[i-1];
        }
        data[0] = dist;

        double smoothDist = 0;
        for(double val : data) smoothDist += val;
        smoothDist /= period;

        slidePos = (int)(smoothDist * 133.869 + 434.94);

        tiltL.setPosition(servoPos);
        tiltR.setPosition(servoPos);

        if(slidePos <= 2600) {
            slideL.setTargetPosition(slidePos);
            slideR.setTargetPosition(slidePos);
        }
        else {
            slideL.setTargetPosition(2600);
            slideR.setTargetPosition(2600);
        }

        lastDist = dist;

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("dist", dist);
        packet.put("smoothDist", smoothDist);
        packet.put("slidePos", slidePos);

        dashboard.sendTelemetryPacket(packet);
    }
}
