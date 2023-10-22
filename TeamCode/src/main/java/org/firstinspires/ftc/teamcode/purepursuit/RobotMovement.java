package org.firstinspires.ftc.teamcode.purepursuit;

import static org.firstinspires.ftc.teamcode.purepursuit.utility.Functions.AngleWrap;
import static org.firstinspires.ftc.teamcode.purepursuit.utility.Functions.lineCircleIntersection;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.purepursuit.utility.CurvePoint;
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
    }

    public void updatePosition() {
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        double[] xs = {(side_length * Math.cos(worldPose.heading) - side_length * Math.sin(worldPose.heading)) + worldPose.x,
                (-side_length * Math.cos(worldPose.heading) - side_length * Math.sin(worldPose.heading)) + worldPose.x,
                (-side_length * Math.cos(worldPose.heading) + side_length * Math.sin(worldPose.heading)) + worldPose.x,
                (side_length * Math.cos(worldPose.heading) + side_length * Math.sin(worldPose.heading)) + worldPose.x};

        double[] ys = {(side_length * Math.sin(worldPose.heading) + side_length * Math.cos(worldPose.heading)) + worldPose.y,
                (-side_length * Math.sin(worldPose.heading) + side_length * Math.cos(worldPose.heading)) + worldPose.y,
                (-side_length * Math.sin(worldPose.heading) - side_length * Math.cos(worldPose.heading)) + worldPose.y,
                (side_length * Math.sin(worldPose.heading) - side_length * Math.cos(worldPose.heading)) + worldPose.y};

        packet.fieldOverlay().fillPolygon(xs, ys).setFill("blue");

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

        dashboard.sendTelemetryPacket(packet);

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
        double distanceToTarget = Math.hypot(targetPos.x - worldPose.x, targetPos.y - worldPose.y);

        double absoluteAngleToTarget = Math.atan2(targetPos.y - worldPose.y, targetPos.x - worldPose.x);

        double relativeAngleToTarget = AngleWrap(absoluteAngleToTarget - worldPose.heading - Math.toRadians(90));

        double relativeXToTarget = Math.cos(relativeAngleToTarget) * distanceToTarget;
        double relativeYToTarget = Math.sin(relativeAngleToTarget) * distanceToTarget;
        double relativeTurnAngle = relativeAngleToTarget - Math.toRadians(180) + targetPos.heading;

        // Preserve shape of vector (basically normalize)
        double movementXPower = relativeXToTarget / (Math.abs(relativeXToTarget) + Math.abs(relativeYToTarget)) * movementSpeed;
        double movementYPower = relativeYToTarget / (Math.abs(relativeYToTarget) + Math.abs(relativeXToTarget)) * movementSpeed;
        double turnPower = Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1) * turnSpeed;

        if (distanceToTarget < 10) turnPower = 0;

        frontLeft.setPower(movementXPower - turnPower + movementYPower);
        frontRight.setPower(movementXPower + turnPower - movementYPower);
        backLeft.setPower(movementXPower - turnPower - movementYPower);
        backRight.setPower(movementXPower + turnPower + movementYPower);
    }

    /**
     * Finds the current point that the robot should be following
     * @param pathPoints List including all points for the lines in the trajectory
     * @param pos Position of the robot
     * @param followRadius Specifies how far away the algorithm looks for points to follow
     * @return CurvePoint specifying the point that the robot should move towards
     */
    public CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point pos, double followRadius) {
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));
        for (int i = 0; i < pathPoints.size() - 1; i++) {
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            ArrayList<Point> intersections = lineCircleIntersection(pos, followRadius, startLine.toPoint(), endLine.toPoint());

            double closestAngle = 100000000;
            for (Point thisIntersection : intersections) { // Find the intersection that's closest to our robot's current orientation
                double angle = Math.atan2(thisIntersection.y - worldPose.y, thisIntersection.x - worldPose.x);
                double deltaAngle = Math.abs(AngleWrap(angle - worldPose.heading));

                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
    }

    public void followCurve(ArrayList<CurvePoint> allPoints, double followAngle) {
        CurvePoint followMe = getFollowPointPath(allPoints, worldPose.toPoint(), allPoints.get(0).followDistance);
        goToPosition(new Pose(followMe.x, followMe.y, followAngle), followMe.moveSpeed, followMe.turnSpeed);
    }
}
