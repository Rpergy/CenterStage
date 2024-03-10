package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;

import java.util.Arrays;

@TeleOp(name="distance slides", group = "tests")
@Config
public class DistanceSlides extends OpMode {
    ModernRoboticsI2cRangeSensor rangeSensor;

    public static double servoPos = 0.56;
    public static int slidePos = 0;

    public static double m = 197;
    public static double b = 775;

    double lastDist = 0;

    int period = 10;

    double[] data;

    FtcDashboard dashboard;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);

        Actuation.setSlides(slidePos);
        Actuation.setTilt(servoPos);

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "dist");

        if (!(Double.valueOf(rangeSensor.getDistance(DistanceUnit.INCH)).isNaN())) lastDist = rangeSensor.getDistance(DistanceUnit.INCH);

        data = new double[period];
        Arrays.fill(data, lastDist);

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

        slidePos = (int)(smoothDist * m + b);

        Actuation.setTilt(servoPos);

        if(slidePos > 700) {
            Actuation.setDepositTilt(ActuationConstants.Deposit.depositTilts[1]);
        }
        else {
            Actuation.setDepositTilt(ActuationConstants.Deposit.intakeTilt);
        }

        if(slidePos <= 2500) {
            Actuation.setSlides(slidePos);
        }
        else {
            Actuation.setSlides(2500);
        }

        lastDist = dist;

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("dist", dist);
        packet.put("smoothDist", smoothDist);
        packet.put("slidePos", slidePos);

        dashboard.sendTelemetryPacket(packet);
    }
}
