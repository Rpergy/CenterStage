package org.firstinspires.ftc.teamcode.tests.other;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;

@TeleOp(name = "weighted drive", group = "tests")
public class WeightedDrive extends OpMode {
    double[] xPos;
    double[] yPos;

    int period = 500;

    FtcDashboard dashboard;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);

        xPos = new double[period];
        yPos = new double[period];

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;

        if (xPos.length - 1 >= 0) System.arraycopy(xPos, 0, xPos, 1, xPos.length - 1);
        xPos[0] = x;

        if (yPos.length - 1 >= 0) System.arraycopy(yPos, 0, yPos, 1, yPos.length - 1);
        yPos[0] = x;

        double xAvg = 0;
        for(double val : xPos) xAvg += val;
        xAvg /= period;

        double yAvg = 0;
        for(double val : yPos) yAvg += val;
        yAvg /= period;

        Actuation.drive(xAvg, yAvg, 0);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("x", x);
        packet.put("y", y);
        packet.put("xAvg", xAvg);
        packet.put("yAvg", yAvg);
        dashboard.sendTelemetryPacket(packet);
    }
}
