package org.firstinspires.ftc.teamcode.purepursuit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.purepursuit.utility.CurvePoint;
import org.firstinspires.ftc.teamcode.purepursuit.utility.Pose;

import java.util.ArrayList;

@Autonomous(group = "Test", name = "PurePursuit")
public class PurePursuit extends OpMode {
    RobotMovement robot;

    @Override
    public void init() {
        robot = new RobotMovement(hardwareMap, new Pose(0, 0, 0));
    }

    @Override
    public void loop() {
        robot.updatePosition();

        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0.0, 0.0, 0.5, 0.5, 50.0));
        allPoints.add(new CurvePoint(100, 0.0, 0.5, 0.5, 50.0));
        allPoints.add(new CurvePoint(0.0, 100, 0.5, 0.5, 50.0));
        //robot.followCurve(allPoints, Math.toRadians(90));

        robot.displayPath(allPoints);
    }
}
