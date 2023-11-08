package org.firstinspires.ftc.teamcode.tests.purepursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.tests.purepursuit.utility.Point;
import org.firstinspires.ftc.teamcode.tests.purepursuit.utility.Pose;

import java.util.ArrayList;

@Autonomous(group = "Test", name = "PurePursuit")
public class PurePursuit extends OpMode {
    RobotMovement robot;

    double moveSpeed;
    double turnSpeed;
    double followDistance;

    @Override
    public void init() {
        robot = new RobotMovement(hardwareMap, new Pose(0, 0, 0));
        moveSpeed = 0.2;
        turnSpeed = 0.9;
        followDistance = 3;
    }

    @Override
    public void loop() {
        robot.updatePosition(telemetry);

        ArrayList<Point> allPoints = new ArrayList<>();
        allPoints.add(new Point(0.0, 0.0));
        allPoints.add(new Point(60, 0));
        allPoints.add(new Point(60, -60));

        robot.followCurve(allPoints, followDistance, moveSpeed, turnSpeed, telemetry);

        robot.display(allPoints, followDistance);
    }
}
