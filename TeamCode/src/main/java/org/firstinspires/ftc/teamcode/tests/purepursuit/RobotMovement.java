package org.firstinspires.ftc.teamcode.tests.purepursuit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tests.purepursuit.utility.MathFunctions;
import org.firstinspires.ftc.teamcode.tests.purepursuit.utility.Point;
import org.firstinspires.ftc.teamcode.tests.purepursuit.utility.Pose;

import java.util.ArrayList;
import java.util.List;

public class RobotMovement {
    DcMotor frontLeft, frontRight, backLeft, backRight;

    public static double wheel_circ, ticksPerRev, track_width, forward_offset;
    Pose worldPose;
    double prev_ticks_left = 0, prev_ticks_right = 0, prev_ticks_back = 0;
    double dx, dy, dtheta;
    double dx_center, dx_perpendicular;
    double side_length = 5;
    double scale;

    double center_multiplier, lateral_multiplier, perpendicular_multiplier;

    FtcDashboard dashboard;

    List<LynxModule> allHubs;

    public RobotMovement(HardwareMap hardwareMap, Pose startPos) {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        wheel_circ = 15.708; // cm
        track_width = 29.53/2; // cm distance between drive wheels
        forward_offset = -14.5; // cm distance from center of robot to perp wheel
        ticksPerRev = 8192;

        lateral_multiplier = 1.033174886;
        perpendicular_multiplier = 1.06;//1.2;
        center_multiplier = 1.08;

        track_width *= lateral_multiplier;

        scale = wheel_circ / ticksPerRev;

        worldPose = startPos;

        dashboard = FtcDashboard.getInstance();
    }

    /**
     * Updates the robot's position based off of encoder values from odometry
     */
    public void updatePosition(Telemetry telemetry) {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        double ticks_left = frontLeft.getCurrentPosition();
        double ticks_right = frontRight.getCurrentPosition();
        double ticks_back = backRight.getCurrentPosition();

        double delta_ticks_left = (ticks_left - prev_ticks_left);
        double delta_ticks_right = (ticks_right - prev_ticks_right);
        double delta_ticks_back = (ticks_back - prev_ticks_back);

        dtheta = ((delta_ticks_left - delta_ticks_right) / track_width) * scale;
        dx_center = ((delta_ticks_left + delta_ticks_right) / 2) * scale * center_multiplier;
        dx_perpendicular = -1 * (delta_ticks_back - (forward_offset * ((delta_ticks_left - delta_ticks_right) / track_width))) * scale * perpendicular_multiplier;

        dx = dx_center * Math.cos(worldPose.heading) - dx_perpendicular * Math.sin(worldPose.heading);
        dy = dx_center * Math.sin(worldPose.heading) + dx_perpendicular * Math.cos(worldPose.heading);

        worldPose.x += dx;
        worldPose.y += dy;
        worldPose.heading += -1 * dtheta;

        prev_ticks_back = backRight.getCurrentPosition();
        prev_ticks_left = frontLeft.getCurrentPosition();
        prev_ticks_right = frontRight.getCurrentPosition();

//        telemetry.addData("X", worldPose.x);
//        telemetry.addData("Y", worldPose.y);
//        telemetry.addData("Heading", worldPose.heading);
//        telemetry.update();
    }

    /**
     * Moves the robot to a specified position
     * @param targetPos Target x, y, and heading for the robot. Heading refers to the preferred angle of the robot
     * @param movementSpeed Motor power multiplier for movement
     * @param turnSpeed Motor power multiplier for turning
     */
    public void goToPosition(Point targetPos, double movementSpeed, double turnSpeed, Telemetry telemetry) {
        double deltaX = targetPos.x - worldPose.x;
        double deltaY = targetPos.y - worldPose.y;

        double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

        double thetaT = 0;

        if (distance != 0)
            thetaT = Math.acos(deltaX/distance);

        double deltaR = MathFunctions.AngleWrap(thetaT - worldPose.heading);

        double movePower = Math.cos(deltaR) * movementSpeed;
        double strafePower = -Math.sin(deltaR) * Math.signum(targetPos.y - worldPose.y) * movementSpeed;
        double turnPower = deltaR/Math.PI * turnSpeed;

//        if (distance <= 10) {
//            movePower = 0;
//            strafePower = 0;
//            turnPower = 0;
//        }

        telemetry.addData("distance", distance);
        telemetry.addData("movePower", movePower);
        telemetry.addData("strafePower", strafePower);
        telemetry.addData("turnPower", turnPower);
        telemetry.update();

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
            intersections.addAll(MathFunctions.lineCircleIntersection(pos, followRadius, startLine, endLine));
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

    /**
     * Makes the robot autonomously follow a curve
     * @param allPoints Points that make up the curve
     * @param followDistance Distance that follow points are detected
     * @param moveSpeed Motor power multiplier for movement
     * @param turnSpeed Motor power multiplier for turning
     */
    public void followCurve(ArrayList<Point> allPoints, double followDistance, double moveSpeed, double turnSpeed, Telemetry telemetry) {
        Point followMe = getFollowPointPath(allPoints, worldPose.toPoint(), followDistance);
        goToPosition(followMe, moveSpeed, turnSpeed, telemetry);
    }

    /**
     * Shows a path of points as well as the robot on the FTC Dashboard
     * @param allPoints List of points to show
     */
    public void display(ArrayList<Point> allPoints, double radius) {
        TelemetryPacket packet = new TelemetryPacket();
        // packet.fieldOverlay().drawImage("centerstageField.jpg", 0, 0, 150, 150);

        packet.fieldOverlay().strokeCircle(worldPose.x, worldPose.y, radius);

        for (int i = 0; i < allPoints.size() - 1; i++) {
            Point point1 = allPoints.get(i);
            Point point2 = allPoints.get(i + 1);
            packet.fieldOverlay().strokeLine(point1.x, point1.y, point2.x, point2.y);
        }

        double[] xs = {(side_length * Math.cos(worldPose.heading) - side_length * Math.sin(worldPose.heading)) + worldPose.x,
                (-side_length * Math.cos(worldPose.heading) - side_length * Math.sin(worldPose.heading)) + worldPose.x,
                (-side_length * Math.cos(worldPose.heading) + side_length * Math.sin(worldPose.heading)) + worldPose.x,
                (side_length * Math.cos(worldPose.heading) + side_length * Math.sin(worldPose.heading)) + worldPose.x,
                Math.cos(worldPose.heading) * side_length + worldPose.x};

        double[] ys = {(side_length * Math.sin(worldPose.heading) + side_length * Math.cos(worldPose.heading)) + worldPose.y,
                (-side_length * Math.sin(worldPose.heading) + side_length * Math.cos(worldPose.heading)) + worldPose.y,
                (-side_length * Math.sin(worldPose.heading) - side_length * Math.cos(worldPose.heading)) + worldPose.y,
                (side_length * Math.sin(worldPose.heading) - side_length * Math.cos(worldPose.heading)) + worldPose.y,
                Math.sin(worldPose.heading) * side_length + worldPose.y};

        packet.fieldOverlay().fillPolygon(xs, ys).setFill("blue");

        packet.fieldOverlay().strokeLine(worldPose.x, worldPose.y, xs[4], ys[4]).setStroke("white");

        dashboard.sendTelemetryPacket(packet);
    }
}
