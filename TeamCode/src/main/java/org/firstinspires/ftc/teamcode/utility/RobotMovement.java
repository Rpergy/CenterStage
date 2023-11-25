package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;
import java.util.List;

public class RobotMovement {
    DcMotor frontLeft, frontRight, backLeft, backRight;

    public Pose robotPose;
    double prev_ticks_left = 0, prev_ticks_right = 0, prev_ticks_back = 0;
    double dx, dy, dtheta;
    double dx_center, dx_perpendicular;
    double side_length = 5;

    double searchIncrease;

    boolean lockOnEnd;

    FtcDashboard dashboard;

    List<LynxModule> allHubs;
    private TelemetryPacket packet;

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

        searchIncrease = 1;

        robotPose = new Pose(startPos);

        lockOnEnd = false;

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

        dtheta = ((delta_ticks_left - delta_ticks_right) / ActuationConstants.Drivetrain.track_width) * ActuationConstants.Drivetrain.scale;
        dx_center = ((delta_ticks_left + delta_ticks_right) / 2) * ActuationConstants.Drivetrain.scale * ActuationConstants.Drivetrain.center_multiplier;
        dx_perpendicular = -1 * (delta_ticks_back - (ActuationConstants.Drivetrain.forward_offset * ((delta_ticks_left - delta_ticks_right) / ActuationConstants.Drivetrain.track_width))) * ActuationConstants.Drivetrain.scale * ActuationConstants.Drivetrain.perpendicular_multiplier;

        dx = dx_center * Math.cos(robotPose.heading) - dx_perpendicular * Math.sin(robotPose.heading);
        dy = dx_center * Math.sin(robotPose.heading) + dx_perpendicular * Math.cos(robotPose.heading);

        robotPose.x += dx;
        robotPose.y += dy;
        robotPose.heading += -1 * dtheta;

        prev_ticks_back = ticks_back;
        prev_ticks_left = ticks_left;
        prev_ticks_right = ticks_right;
    }

    /**
     * Moves the robot to a specified position
     * @param targetPos Target x, y, and heading for the robot. Heading refers to the preferred angle of the robot
     * @param movementSpeed Motor power multiplier for movement
     * @param turnSpeed Motor power multiplier for turning
     */
    public void goToPosition(Point targetPos, double movementSpeed, double turnSpeed) {
        double deltaX = targetPos.x - robotPose.x;
        double deltaY = targetPos.y - robotPose.y;

        double tTheta = Math.atan2(deltaY, deltaX);

        goToPose(new Pose(targetPos, tTheta), movementSpeed, turnSpeed);
    }

    public void goToPose(Pose targetPose, double movementSpeed, double turnSpeed) {
        TelemetryPacket packet = new TelemetryPacket();

        double deltaX = targetPose.x - robotPose.x;
        double deltaY = targetPose.y - robotPose.y;

        double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

        double deltaTheta = MathFunctions.AngleWrap(targetPose.heading - robotPose.heading);

        double turnPower = deltaTheta/Math.PI * turnSpeed;

        if (Math.abs(deltaTheta) <= Math.toRadians(5)) {
            turnPower = 0;
        }
        else if (deltaTheta > 0) {
            turnPower = Math.max(turnPower, ActuationConstants.Autonomous.minTurnSpeed);
        }
        else {
            turnPower = Math.min(turnPower, -ActuationConstants.Autonomous.minTurnSpeed);
        }

        double m1 = (Math.tanh(deltaY * ActuationConstants.Autonomous.accelMult) * Math.sin(robotPose.heading));
        double m2 = (Math.tanh(deltaX * ActuationConstants.Autonomous.accelMult) * Math.cos(robotPose.heading));

        double s1 = (-Math.tanh(deltaY * ActuationConstants.Autonomous.accelMult) * Math.cos(robotPose.heading));
        double s2 = (Math.tanh(deltaX * ActuationConstants.Autonomous.accelMult) * Math.sin(robotPose.heading));

        double movePower = (m1 * Math.abs(m1) + m2 * Math.abs(m2)) * movementSpeed;
        double strafePower =  (s1 * Math.abs(s1) + s2 * Math.abs(s2)) * movementSpeed;

        if (distance <= 4) {
            movePower = 0;
            strafePower = 0;
        }

        packet.put("move", movePower);
        packet.put("turn", turnPower);
        packet.put("strafe", strafePower);

        frontLeft.setPower(-movePower + turnPower - strafePower);
        frontRight.setPower(-movePower - turnPower + strafePower);
        backLeft.setPower(-movePower + turnPower + strafePower);
        backRight.setPower(-movePower - turnPower - strafePower);

        dashboard.sendTelemetryPacket(packet);
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

        while (intersections.size() == 0) {
            for (int i = 0; i < pathPoints.size() - 1; i++) {
                Point startLine = pathPoints.get(i);
                Point endLine = pathPoints.get(i + 1);
                intersections.addAll(MathFunctions.lineCircleIntersection(pos, followRadius, startLine, endLine));
            }
            followRadius += searchIncrease;
        }

        double closestAngle = 10000;
        for (int i = 0; i < intersections.size(); i++) {
            Point thisIntersection = intersections.get(i);
            double deltaX = thisIntersection.x - robotPose.x;
            double deltaY = thisIntersection.y - robotPose.y;
            double deltaR = Math.abs(Math.atan2(deltaY, deltaX) - robotPose.heading);

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
    public void followCurve(ArrayList<Point> allPoints, double followDistance, double moveSpeed, double turnSpeed) {
        Point followMe = getFollowPointPath(allPoints, robotPose.toPoint(), followDistance);
        if (followMe.inRange(allPoints.get(allPoints.size()-1), 1.0) || lockOnEnd) {
            goToPosition(allPoints.get(allPoints.size()-1), moveSpeed, turnSpeed);
            lockOnEnd = true;
        }
        else
            goToPosition(followMe, moveSpeed, turnSpeed);
    }

    /**
     * Shows a path of points as well as the robot on the FTC Dashboard
     * @param allPoints List of points to show
     */
    public void display(ArrayList<Point> allPoints, double radius) {
        TelemetryPacket packet = new TelemetryPacket();
        // packet.fieldOverlay().drawImage("centerstageField.jpg", 0, 0, 150, 150);

        packet.fieldOverlay().strokeCircle(robotPose.x, robotPose.y, radius);

        ArrayList<Point> intersections = new ArrayList<>();

        for (int i = 0; i < allPoints.size() - 1; i++) {
            Point startLine = allPoints.get(i);
            Point endLine = allPoints.get(i + 1);
            intersections.addAll(MathFunctions.lineCircleIntersection(robotPose.toPoint(), radius, startLine, endLine));
        }

        for(int i = 0; i < intersections.size(); i++) {
            packet.fieldOverlay().strokeCircle(intersections.get(i).x, intersections.get(i).y, 2);
        }

        Point followMe = getFollowPointPath(allPoints, robotPose.toPoint(), radius);
        packet.fieldOverlay().setStroke("red");
        packet.fieldOverlay().strokeCircle(followMe.x, followMe.y, 2);

        packet.fieldOverlay().setStroke("black");
        for (int i = 0; i < allPoints.size() - 1; i++) {
            Point point1 = allPoints.get(i);
            Point point2 = allPoints.get(i + 1);
            packet.fieldOverlay().strokeLine(point1.x, point1.y, point2.x, point2.y);
        }

        double[] xs = {(side_length * Math.cos(robotPose.heading) - side_length * Math.sin(robotPose.heading)) + robotPose.x,
                (-side_length * Math.cos(robotPose.heading) - side_length * Math.sin(robotPose.heading)) + robotPose.x,
                (-side_length * Math.cos(robotPose.heading) + side_length * Math.sin(robotPose.heading)) + robotPose.x,
                (side_length * Math.cos(robotPose.heading) + side_length * Math.sin(robotPose.heading)) + robotPose.x,
                Math.cos(robotPose.heading) * side_length + robotPose.x};

        double[] ys = {(side_length * Math.sin(robotPose.heading) + side_length * Math.cos(robotPose.heading)) + robotPose.y,
                (-side_length * Math.sin(robotPose.heading) + side_length * Math.cos(robotPose.heading)) + robotPose.y,
                (-side_length * Math.sin(robotPose.heading) - side_length * Math.cos(robotPose.heading)) + robotPose.y,
                (side_length * Math.sin(robotPose.heading) - side_length * Math.cos(robotPose.heading)) + robotPose.y,
                Math.sin(robotPose.heading) * side_length + robotPose.y};

        packet.fieldOverlay().fillPolygon(xs, ys).setFill("blue");

        packet.fieldOverlay().setStroke("white");
        packet.fieldOverlay().strokeLine(robotPose.x, robotPose.y, xs[4], ys[4]);

        dashboard.sendTelemetryPacket(packet);
    }
}
