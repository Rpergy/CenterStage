package org.firstinspires.ftc.teamcode.tests.purepursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utility.CurvePoint;
import org.firstinspires.ftc.teamcode.utility.Point;
import org.firstinspires.ftc.teamcode.utility.Pose;
import org.firstinspires.ftc.teamcode.utility.RobotMovement;

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
        turnSpeed = 0.4;
        followDistance = 10;
    }

    @Override
    public void loop() {
        robot.updatePosition(telemetry);

        ArrayList<Point> allPoints = new ArrayList<>();
        allPoints.add(new Point(-50, 25));
        allPoints.add(new Point(0, 25));

        robot.goToPose(new Pose(0, 25, Math.PI/2), moveSpeed, turnSpeed);

//        robot.followCurve(allPoints, followDistance, moveSpeed, turnSpeed);

        robot.display(allPoints, followDistance);
    }
}
