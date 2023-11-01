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
        moveSpeed = 1;
        turnSpeed = 1;
    }

    @Override
    public void loop() {
        robot.updatePosition();

        ArrayList<Point> allPoints = new ArrayList<>();
        allPoints.add(new Point(0.0, 0.0));
        allPoints.add(new Point(100, 0.0));
        allPoints.add(new Point(0.0, 100));

        robot.followCurve(allPoints, followDistance, moveSpeed, turnSpeed);

        robot.display(allPoints);
    }
}
