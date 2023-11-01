package org.firstinspires.ftc.teamcode.purepursuit;

import static org.firstinspires.ftc.teamcode.purepursuit.utility.MathFunctions.lineCircleIntersection;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.purepursuit.utility.MathFunctions;
import org.firstinspires.ftc.teamcode.purepursuit.utility.Point;
import org.firstinspires.ftc.teamcode.purepursuit.utility.Pose;

import java.util.ArrayList;

public class RobotMovement {
    DcMotor frontLeft, frontRight, backLeft, backRight;

    public static double wheel_circ, ticksPerRev, track_width, forward_offset;
    Pose worldPose;
    double prev_ticks_left = 0, prev_ticks_right = 0, prev_ticks_back = 0;
    double dx, dy, dtheta;
    double dx_center, dx_perpendicular;
    double side_length = 5;
    double scale;
    public static double lateral_offset;

    FtcDashboard dashboard;

    public RobotMovement(HardwareMap hardwareMap, Pose startPos) {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        wheel_circ = 15.708; // cm
        track_width = 29.53/2; // cm distance between drive wheels
        forward_offset = -14.5; // cm distance from center of robot to perp wheel
        ticksPerRev = 8192;

        lateral_offset = 0.5;

        scale = wheel_circ / ticksPerRev;

        worldPose = startPos;

        dashboard = FtcDashboard.getInstance();
    }

    public void updatePosition() {
        double delta_ticks_left = (frontLeft.getCurrentPosition() - prev_ticks_left);
        double delta_ticks_right = (frontRight.getCurrentPosition() - prev_ticks_right);
        double delta_ticks_back = (backRight.getCurrentPosition() - prev_ticks_back);

        dtheta = ((delta_ticks_left - delta_ticks_right) / track_width) * scale;
        dx_center = ((delta_ticks_left + delta_ticks_right) / 2) * scale;
        dx_perpendicular = (delta_ticks_back - (forward_offset * ((delta_ticks_left - delta_ticks_right) / track_width))) * scale * lateral_offset;

        dx = dx_center * Math.cos(worldPose.heading) - dx_perpendicular * Math.sin(worldPose.heading);
        dy = dx_center * Math.sin(worldPose.heading) + dx_perpendicular * Math.cos(worldPose.heading);

        worldPose.x += dx;
        worldPose.y += dy;
        worldPose.heading += -1 * dtheta;

        prev_ticks_back = backRight.getCurrentPosition();
        prev_ticks_left = frontLeft.getCurrentPosition();
        prev_ticks_right = frontRight.getCurrentPosition();
    }

    /**
     * Moves the robot to a specified position
     * @param targetPos Target x, y, and heading for the robot. Heading refers to the preferred angle of the robot
     * @param movementSpeed Motor power
     * @param turnSpeed Robot turn speed
     */
    public void goToPosition(Pose targetPos, double movementSpeed, double turnSpeed) {
        double deltaX = targetPos.x - worldPose.x;
        double deltaY = targetPos.y - worldPose.y;

        double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

        double thetaT = 0;

        if (distance != 0)
            thetaT = Math.acos(deltaX/distance);

        double deltaR = MathFunctions.AngleWrap(thetaT - worldPose.heading);

        double movePower = Math.cos(deltaR) * movementSpeed;
        double strafePower = Math.sin(deltaR) * Math.signum(targetPos.y - worldPose.y) * movementSpeed;
        double turnPower = (thetaT/Math.PI) * (thetaT - worldPose.heading) * turnSpeed;

        frontLeft.setPower(movePower - turnPower + strafePower);
        frontRight.setPower(movePower + turnPower - strafePower);
        backLeft.setPower(movePower - turnPower - strafePower);
        backRight.setPower(movePower + turnPower + strafePower);
    }

    /**
     * Finds the current point that the robot should be following
     * @param pathPoints List including all points for the lines in the trajectory
     * @param pos Position of the robot
     * @param followRadius Specifies how far away the algorithm looks for points to follow
     * @return CurvePoint specifying the point that the robot should move towards
     */
    public Point getFollowPointPath(ArrayList<Point> pathPoints, Point pos, double followRadius) {
        Point followMe = new Point(pathPoints.get(0));

        ArrayList<Point> intersections = new ArrayList<>();

        for (int i = 0; i < pathPoints.size() - 1; i++) {
            Point startLine = pathPoints.get(i);
            Point endLine = pathPoints.get(i + 1);
            intersections.addAll(lineCircleIntersection(pos, followRadius, startLine, endLine));
        }
        double closestAngle = 10000;
        for (int i = 0; i < intersections.size(); i++) {
            Point thisIntersection = intersections.get(i);
            double deltaX = thisIntersection.x - worldPose.x;
            double deltaY = thisIntersection.y - worldPose.y;
            double deltaR = Math.abs(Math.atan2(deltaY, deltaX) - worldPose.heading);

            if (deltaR < closestAngle) {
                closestAngle = deltaR;
                followMe.x = thisIntersection.x;
                followMe.y = thisIntersection.y;
            }
        }

        return followMe;
    }

    public void followCurve(ArrayList<Point> allPoints, double followDistance, double moveSpeed, double turnSpeed) {
        Point followMe = getFollowPointPath(allPoints, worldPose.toPoint(), followDistance);
        goToPosition(new Pose(followMe.x, followMe.y, followDistance), moveSpeed, turnSpeed);
    }

    /**
     * Shows a path of points on the FTC Dashboard
     * @param allPoints List of points to show
     */
    public void displayPath(ArrayList<Point> allPoints) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().drawImage("centerstageField.jpg", 0, 0, 150, 150);

        for (int i = 0; i < allPoints.size() - 1; i++) {
            Point point1 = allPoints.get(i);
            Point point2 = allPoints.get(i + 1);
            packet.fieldOverlay().strokeLine(point1.x, point1.y, point2.x, point2.y);
        }

        double[] xs = {(side_length * Math.cos(worldPose.heading) - side_length * Math.sin(worldPose.heading)) + worldPose.x,
                (-side_length * Math.cos(worldPose.heading) - side_length * Math.sin(worldPose.heading)) + worldPose.x,
                (-side_length * Math.cos(worldPose.heading) + side_length * Math.sin(worldPose.heading)) + worldPose.x,
                (side_length * Math.cos(worldPose.heading) + side_length * Math.sin(worldPose.heading)) + worldPose.x};

        double[] ys = {(side_length * Math.sin(worldPose.heading) + side_length * Math.cos(worldPose.heading)) + worldPose.y,
                (-side_length * Math.sin(worldPose.heading) + side_length * Math.cos(worldPose.heading)) + worldPose.y,
                (-side_length * Math.sin(worldPose.heading) - side_length * Math.cos(worldPose.heading)) + worldPose.y,
                (side_length * Math.sin(worldPose.heading) - side_length * Math.cos(worldPose.heading)) + worldPose.y};

        packet.fieldOverlay().fillPolygon(xs, ys).setFill("blue");

        dashboard.sendTelemetryPacket(packet);
    }
}
