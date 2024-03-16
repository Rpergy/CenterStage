package org.firstinspires.ftc.teamcode.tests.other;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;

@TeleOp(name = "weighted drive", group = "tests")
public class WeightedDrive extends OpMode {
    double[] movePos;
    double[] strafePos;

    int period = 100;

    FtcDashboard dashboard;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);

        movePos = new double[period];
        strafePos = new double[period];

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        double move = 0;
        double strafe = gamepad1.right_stick_x;

        if(gamepad1.left_trigger > 0.1) move = gamepad1.left_trigger;
        else if (gamepad1.right_trigger > 0.1) move = -gamepad1.right_trigger;

        if (movePos.length - 1 >= 0) System.arraycopy(movePos, 0, movePos, 1, movePos.length - 1);
        movePos[0] = move;

        if (strafePos.length - 1 >= 0) System.arraycopy(strafePos, 0, strafePos, 1, strafePos.length - 1);
        strafePos[0] = strafe;

        double moveAvg = 0;
        for(double val : movePos) moveAvg += val;
        moveAvg /= period;

        double strafeAvg = 0;
        for(double val : strafePos) strafeAvg += val;
        strafeAvg /= period;

        Actuation.drive(moveAvg, gamepad1.left_stick_x, strafeAvg);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("move", move);
        packet.put("strafe", strafe);
        packet.put("moveAvg", moveAvg);
        packet.put("strafeAvg", strafeAvg);
        dashboard.sendTelemetryPacket(packet);
    }
}
