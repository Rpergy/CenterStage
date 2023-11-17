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
        robot = new RobotMovement(hardwareMap, new Pose(-50, 25, 0));
        moveSpeed = 0.3;
        turnSpeed = 0.7;
        followDistance = 10;
    }

    @Override
    public void loop() {
        robot.updatePosition(telemetry);

        ArrayList<Point> allPoints = new ArrayList<>();
        allPoints.add(new Point(-50, 25));
        allPoints.add(new Point(0, 25));
        allPoints.add(new Point(0, -40));
        allPoints.add(new Point(40, -40));
        allPoints.add(new Point(40, 25));

        robot.followCurve(allPoints, followDistance, moveSpeed, turnSpeed);

        robot.display(allPoints, followDistance);
    }
}
