package org.firstinspires.ftc.teamcode.tests.purepursuit;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.RobotMovement;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;

@TeleOp(name="Waypoint Drive")
public class WaypointDrive extends OpMode {
    RobotMovement robot;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
        robot = new RobotMovement(hardwareMap, new Pose(50, -50, 0));
    }

    @Override
    public void loop() {
        robot.updatePosition();

        FieldConstants.Waypoint startPose = FieldConstants.Waypoint.BLUE_WING;
        FieldConstants.Waypoint endPose = FieldConstants.Waypoint.BLUE_CANVAS;

        ArrayList<Pose> path = buildPath(startPose.ordinal(), endPose.ordinal());

        robot.incrementPoseCurve(path, ActuationConstants.Autonomous.followDistance, ActuationConstants.Autonomous.moveSpeed, ActuationConstants.Autonomous.turnSpeed);
        robot.displayPoses(path, ActuationConstants.Autonomous.followDistance);
    }

    public ArrayList<Pose> buildPath(int startPose, int endPose) {
        ArrayList<Integer> verticies = new ArrayList<>();
        ArrayList<Double> dist = new ArrayList<>();
        ArrayList<Integer> prev = new ArrayList<>();

        ArrayList<Integer> path = new ArrayList<>();

        for (int i = 0; i < FieldConstants.waypointConnections.length; i++) {
            dist.add(Double.MAX_VALUE);
            prev.add(Integer.MIN_VALUE);
            verticies.add(i);
        }

        dist.set(startPose, 0.0);

        while (verticies.size() != 0) {
            double minDistance = Double.MAX_VALUE;
            int u = 0;

            for (int i = 0; i < dist.size(); i++) {
                if (dist.get(i) < minDistance && verticies.contains(i)) {
                    minDistance = dist.get(i);
                    u = i;
                }
            }

            if (u == endPose) {
                if (prev.get(u) != Integer.MIN_VALUE || u == startPose) {
                    while (u != Integer.MIN_VALUE) {
                        path.add(0, u);
                        u = prev.get(u);
                    }
                }
                break;
            }

            verticies.remove(u);

            ArrayList<Integer> neighbors = new ArrayList<>();

            for (int i = 0; i < FieldConstants.waypointConnections.length; i++) {
                if (FieldConstants.waypointConnections[u][i] != 0) neighbors.add(i);
            }

            for (int i = 0; i < neighbors.size(); i++) {
                int n = neighbors.get(i);

                if (verticies.contains(n)) {
                    double alt = dist.get(u) + FieldConstants.waypointConnections[u][n];
                    if(alt < dist.get(n)) {
                        dist.set(n, alt);
                        prev.set(n, u);
                    }
                }
            }
        }

        ArrayList<Pose> waypointPath = new ArrayList<>();

        for (int i = 0; i < path.size(); i++) {
            waypointPath.add(FieldConstants.waypointPositions[path.get(i)]);
        }

        return waypointPath;
    }
}
