package org.firstinspires.ftc.teamcode.tests.purepursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;
import org.firstinspires.ftc.teamcode.utility.RobotMovement;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;

@Autonomous(group = "Test", name = "PurePursuit")
public class PurePursuit extends OpMode {
    RobotMovement robot;

    @Override
    public void init() {
        robot = new RobotMovement(hardwareMap, ActuationConstants.Autonomous.robotStart);
    }

    @Override
    public void loop() {
        robot.updatePosition(telemetry);

        ArrayList<Point> allPoints = new ArrayList<>();
        allPoints.add(new Point(-50, 25));
        allPoints.add(new Point(0, 45));

        robot.goToPose(new Pose(0, 45, Math.PI/2), ActuationConstants.Autonomous.moveSpeed, ActuationConstants.Autonomous.turnSpeed);

//        robot.followCurve(allPoints, ActuationConstants.Autonomous.followDistance, ActuationConstants.Autonomous.moveSpeed, ActuationConstants.Autonomous.turnSpeed);

        robot.display(allPoints, ActuationConstants.Autonomous.followDistance);
    }
}
