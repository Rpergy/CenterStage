package org.firstinspires.ftc.teamcode.tests.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.MathFunctions;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;
import java.util.List;

public class DodgeMovement {
    VoltageSensor voltageSensor;
    DcMotor frontLeft, frontRight, backLeft, backRight;

    public Pose robotPose;
    double prev_ticks_left = 0, prev_ticks_right = 0, prev_ticks_back = 0;
    double dx, dy, dtheta;
    double dx_center, dx_perpendicular;
    double side_length = 7;

    double searchIncrease;

    boolean lockOnEnd;

    FtcDashboard dashboard;

    List<LynxModule> allHubs;
    private TelemetryPacket packet;

    int targetControlPoint;

    double movePower, turnPower, strafePower;

    public DodgeMovement(HardwareMap hardwareMap, Pose startPos) {
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

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        searchIncrease = 1;

        robotPose = new Pose(startPos);

        lockOnEnd = false;

        dashboard = FtcDashboard.getInstance();

        targetControlPoint = 0;
    }

    /**
     * Updates the robot's pose based off of encoder values from odometry
     */
    public void updatePosition() {
        TelemetryPacket packet = new TelemetryPacket();
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
        dx_center = ((delta_ticks_left + delta_ticks_right) / 2) * ActuationConstants.Drivetrain.scale * ActuationConstants.Drivetrain.centerMultiplier;
        dx_perpendicular = -1 * (delta_ticks_back - (ActuationConstants.Drivetrain.forward_offset * ((delta_ticks_left - delta_ticks_right) / ActuationConstants.Drivetrain.track_width))) * ActuationConstants.Drivetrain.scale * ActuationConstants.Drivetrain.perpendicularMultiplier;

//        pose exponential terribleness
//        a = robotPose.heading
//        b = dtheta
//        c = dx_center
//        d = dx_perpendicular
        if(dtheta != 0) {
            dx = (dx_center * (Math.sin(dtheta) * Math.cos(robotPose.heading) - Math.sin(robotPose.heading) * (-Math.cos(dtheta) + 1)) + dx_perpendicular * (Math.cos(robotPose.heading) * (Math.cos(dtheta) - 1) - Math.sin(dtheta) * Math.sin(robotPose.heading))) / dtheta;
            dy = (dx_center * (Math.sin(dtheta) * Math.sin(robotPose.heading) + Math.cos(robotPose.heading) * (-Math.cos(dtheta) + 1)) + dx_perpendicular * (Math.sin(robotPose.heading) * (Math.cos(dtheta) - 1) + Math.sin(dtheta) * Math.cos(robotPose.heading))) / dtheta;
        }
        else {
            dx = dx_center * Math.cos(robotPose.heading) - dx_perpendicular * Math.sin(robotPose.heading);
            dy = dx_center * Math.sin(robotPose.heading) + dx_perpendicular * Math.cos(robotPose.heading);
        }
        robotPose.x += -1 * dx;
        robotPose.y += -1 * dy;
        robotPose.heading += dtheta;

        prev_ticks_back = ticks_back;
        prev_ticks_left = ticks_left;
        prev_ticks_right = ticks_right;
        packet.put("dx", dx);
        packet.put("dy", dy);
        packet.put("dtheta", dtheta);
//        dashboard.sendTelemetryPacket(packet);
    }

    /**
     * Moves the robot to a specified position
     * @param targetPos Target x and y for the robot.
     * @param movementSpeed Motor power multiplier for movement
     * @param turnSpeed Motor power multiplier for turning
     */
    public void goToPosition(Point targetPos, double movementSpeed, double turnSpeed) {
        double deltaX = targetPos.x - robotPose.x;
        double deltaY = targetPos.y - robotPose.y;

        double tTheta = Math.atan2(deltaY, deltaX);

        goToPose(new Pose(targetPos, tTheta), movementSpeed, turnSpeed);
    }

    /**
     * Moves the robot to a specified pose
     * @param targetPose Target x, y, and heading for the robot. Heading refers to the preferred angle of the robot
     * @param movementSpeed Motor power multiplier for movement
     * @param turnSpeed Motor power multiplier for turning
     */
    public void goToPose(Pose targetPose, double movementSpeed, double turnSpeed) {
        TelemetryPacket packet = new TelemetryPacket();

        double deltaX = targetPose.x - robotPose.x;
        double deltaY = targetPose.y - robotPose.y;

        double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

        double deltaTheta = MathFunctions.AngleWrap(targetPose.heading - robotPose.heading);

        turnPower = deltaTheta/Math.PI * turnSpeed;

//        if (Math.abs(deltaTheta) <= Math.toRadians(1)) {
//            turnPower = 0;
//        }
        if (deltaTheta > 0) {
            turnPower = Math.max(turnPower, ActuationConstants.Autonomous.minTurnSpeed);
        }
        else {
            turnPower = Math.min(turnPower, -ActuationConstants.Autonomous.minTurnSpeed);
        }

        double m1 = (Math.tanh(deltaY * ActuationConstants.Autonomous.moveAccelMult) * Math.sin(robotPose.heading));
        double m2 = (Math.tanh(deltaX * ActuationConstants.Autonomous.moveAccelMult) * Math.cos(robotPose.heading));

        double s1 = (-Math.tanh(deltaY * ActuationConstants.Autonomous.strafeAccelMult) * Math.cos(robotPose.heading));
        double s2 = (Math.tanh(deltaX * ActuationConstants.Autonomous.strafeAccelMult) * Math.sin(robotPose.heading));

        movePower = (m1 * Math.abs(m1) + m2 * Math.abs(m2)) * movementSpeed;
        strafePower =  (s1 * Math.abs(s1) + s2 * Math.abs(s2)) * movementSpeed;

//        if (distance <= 0.5) {
//            movePower = 0;
//            strafePower = 0;
//        }

        packet.put("move", movePower);
        packet.put("turn", turnPower);
        packet.put("strafe", strafePower);
        packet.put("deltaTheta", deltaTheta);

        double v1 = -movePower + turnPower - strafePower;
        double v2 = -movePower - turnPower + strafePower;
        double v3 = -movePower + turnPower + strafePower;
        double v4 = -movePower - turnPower - strafePower;

        double voltageComp = 12 / voltageSensor.getVoltage();

        frontLeft.setPower(v1 * voltageComp);
        frontRight.setPower(v2 * voltageComp);
        backLeft.setPower(v3 * voltageComp);
        backRight.setPower(v4 * voltageComp);

        dashboard.sendTelemetryPacket(packet);
    }

    /**
     * Finds the current point that the robot should be following
     * @param pathPoints List including all points for the lines in the trajectory
     * @param pos Position of the robot
     * @param followRadius Specifies how far away the algorithm looks for points to follow
     * @return Point specifying the point that the robot should move towards
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
     * Finds the current pose that the robot should be following
     * @param pathPoints List including all poses for the lines in the trajectory
     * @param pos Pose of the robot
     * @param followRadius Specifies how far away the algorithm looks for poses to follow
     * @return Pose specifying the pose that the robot should move towards
     */
    public Pose getFollowPosePath(ArrayList<Pose> pathPoints, Pose pos, double followRadius) {
        Pose followMe = new Pose(pathPoints.get(0));

        ArrayList<Pose> intersections = new ArrayList<>();

//        iteratively increasing followRadius until valid line circle intersection is found(?)
        while (intersections.size() == 0) {
            for (int i = 0; i < pathPoints.size() - 1; i++) {
                Pose startLine = pathPoints.get(i);
                Pose endLine = pathPoints.get(i + 1);
                intersections.addAll(MathFunctions.lineCircleIntersection(pos.toPoint(), followRadius, startLine, endLine));
            }
            followRadius += searchIncrease;
        }
        double minDistance = Integer.MAX_VALUE;
        for (int i = 0; i < intersections.size(); i++) {
            Pose thisIntersection = intersections.get(i);
            double dist = MathFunctions.distance(thisIntersection.toPoint(), pathPoints.get(targetControlPoint).toPoint());
            if (dist < minDistance) {
                followMe = new Pose(thisIntersection);
                minDistance = dist;
            }
        }

        return followMe;
    }

    /**
     * Makes the robot autonomously follow a curve of points
     * @param allPoints Points that make up the curve
     * @param followDistance Distance that follow points are detected
     * @param moveSpeed Motor power multiplier for movement
     * @param turnSpeed Motor power multiplier for turning
     */
    public void incrementPointCurve(ArrayList<Point> allPoints, double followDistance, double moveSpeed, double turnSpeed) {
        Point followMe = getFollowPointPath(allPoints, robotPose.toPoint(), followDistance);
        if (followMe.withinRange(allPoints.get(allPoints.size()-1), 1.0) || lockOnEnd && targetControlPoint == allPoints.size()-1) {
            goToPosition(allPoints.get(allPoints.size()-1), moveSpeed, turnSpeed);
            lockOnEnd = true;
        }
        else
            goToPosition(followMe, moveSpeed, turnSpeed);
    }

    /**
     * Makes the robot autonomously follow a curve of poses
     * Unlike in RobotMovement, this takes into account potential obstacles from camera output
     * @param allPoints Poses that make up the curve
     * @param followDistance Distance that follow points are detected
     * @param moveSpeed Motor power multiplier for movement
     * @param turnSpeed Motor power multiplier for turning
     */
    public void incrementPoseCurve(ArrayList<Pose> allPoints, double followDistance, double moveSpeed, double turnSpeed) {
        double distanceToTarget = MathFunctions.distance(robotPose.toPoint(), allPoints.get(targetControlPoint).toPoint());

//        only move along the point list if we are closer to the target than the follow distance
//        and if targetControlPoint is less than the last point in the list
//        targetControlPoint is an index into the list
//        since it starts at zero this condition is just stopping incrementing the pose curve list
//        when we reach the end of it so we don't get an out of bounds exception
        if (distanceToTarget <= followDistance && targetControlPoint < allPoints.size()-1) {
            targetControlPoint++;
        }

        populateDodgePoses(allPoints);

        Pose followMe = getFollowPosePath(allPoints, robotPose, followDistance);
        if (followMe.withinRange(allPoints.get(allPoints.size()-1), 1.0) || lockOnEnd && targetControlPoint == allPoints.size()-1) {
            goToPose(allPoints.get(allPoints.size()-1), moveSpeed, turnSpeed);
            lockOnEnd = true;
        }
        else {
            goToPose(followMe, moveSpeed, turnSpeed);
        }
    }

    public void followPoseCurve(Telemetry tel, ArrayList<Pose> allPoints, double followDistance, double moveSpeed, double turnSpeed) {
//        while (targetControlPoint != allPoints.size()-1 || MathFunctions.distance(robotPose, allPoints.get(allPoints.size()-1)) > 1.9) {
//        || 0.9 > MathFunctions.cosineDistance(robotPose.toPoint(), allPoints.get(allPoints.size()-1).toPoint())
        double dist = MathFunctions.distance(robotPose.toPoint(), allPoints.get(allPoints.size()-1).toPoint());
        double rotDist = Math.abs(robotPose.heading - allPoints.get(allPoints.size()-1).heading);
        while (dist > 0.45 || rotDist > Math.toRadians(2)) {
            displayPoses(allPoints, followDistance);

            updatePosition();
            incrementPoseCurve(allPoints, followDistance, moveSpeed, turnSpeed);

            dist = MathFunctions.distance(robotPose.toPoint(), allPoints.get(allPoints.size()-1).toPoint());
            rotDist = Math.abs(robotPose.heading - allPoints.get(allPoints.size()-1).heading);

            tel.addData("dist to end", dist);
            tel.addData("rot to end", rotDist);
            tel.update();
        }

        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);

        targetControlPoint = 0;
        lockOnEnd = false;
    }

    /**
     * Shows a path of points as well as the robot on the FTC Dashboard
     * @param allPoints List of points to show
     */
    public void displayPoints(ArrayList<Point> allPoints, double radius) {
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

    /**
     * Shows a path of poses as well as the robot's position on the FTC Dashboard
     * @param allPoints List of points to show
     */
    public void displayPoses(ArrayList<Pose> allPoints, double radius) {
        TelemetryPacket packet = new TelemetryPacket();
        // packet.fieldOverlay().drawImage("centerstageField.jpg", 0, 0, 150, 150);

        packet.fieldOverlay().strokeCircle(robotPose.x, robotPose.y, radius);

        ArrayList<Pose> intersections = new ArrayList<>();

        for (int i = 0; i < allPoints.size() - 1; i++) {
            Pose startLine = allPoints.get(i);
            Pose endLine = allPoints.get(i + 1);
            intersections.addAll(MathFunctions.lineCircleIntersection(robotPose.toPoint(), radius, startLine, endLine));
        }

        for(int i = 0; i < intersections.size(); i++) {
            packet.fieldOverlay().strokeCircle(intersections.get(i).x, intersections.get(i).y, 2);
        }

        Pose followMe = getFollowPosePath(allPoints, robotPose, radius);
        packet.fieldOverlay().setStroke("red");
        packet.fieldOverlay().strokeCircle(followMe.x, followMe.y, 2);

        packet.fieldOverlay().setStroke("black");
        for (int i = 0; i < allPoints.size() - 1; i++) {
            Pose point1 = allPoints.get(i);
            Pose point2 = allPoints.get(i + 1);
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

        packet.put("move", movePower);
        packet.put("strafe", strafePower);
        packet.put("turn", turnPower);
        packet.put("target point", targetControlPoint);
        packet.put("locked on end", lockOnEnd);

        dashboard.sendTelemetryPacket(packet);
    }

    public void displayPosition(double radius){
        TelemetryPacket packet = new TelemetryPacket();
        // packet.fieldOverlay().drawImage("centerstageField.jpg", 0, 0, 150, 150);

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

    public void populateDodgePoses(ArrayList<Pose> allPoints) {
        // EVERYTHING UNTIL THE NEXT LARGE SPACE NEEDS TO BE UPDATED TO WORK WITH OCCUPANCY MAP...
        // DEPENDING ON HOW QUERYING WITH MAP WORKS, EITHER CHECK FOR INTERSECTS ALONG A LINE OR EVERY r/2 DISTANCE OR 1/2 the segment length if smaller than r/2

        // p1 and p2 should be obtained from allPoints, but need to account for case when the current and current-1 pose are not sufficiently far away
        Point p1 = new Point(allPoints.get(targetControlPoint - 1));
        Point p2 = new Point(allPoints.get(targetControlPoint));

        // should obtain intersects from occupancy map, need to account for driver recreation (organic curves) and smaller than r/2 segments
        // use this as an arraylist or just call again from new starting point?
        Point intersect = findIntersect(p1, p2);





        double radius = 9 * Math.sqrt(2);

        double denominator = p2.x - p1.x;
        double slope = (denominator == 0) ? Double.MAX_VALUE : (double)(p2.y - p1.y) / denominator;
        double perpendicularSlope = (slope == 0) ? Double.MAX_VALUE : -1 / slope;
        double slopeSquared = slope * slope;

        // the following is a derived formula, split into parts for efficiency
        double part1 = slope * (-slope * (-2 * intersect.x - (2 * intersect.x) / slopeSquared));
        double part2 = 2 * radius * Math.sqrt(1 + slopeSquared);
        double part3 = 2 * (slopeSquared + 1);

        Point[] possiblePoints = {
                new Point((part1 + part2) / part3, 0),
                new Point((part1 - part2) / part3, 0)
        };

        possiblePoints[0].y = (perpendicularSlope) * (possiblePoints[0].x - intersect.x) + intersect.y;
        possiblePoints[1].y = (perpendicularSlope) * (possiblePoints[1].x - intersect.x) + intersect.y;

        double distance0 = Math.abs(slope * possiblePoints[0].x - possiblePoints[0].y - slope * p1.x + p1.y) / Math.sqrt(slopeSquared + 1);
        double distance1 = Math.abs(slope * possiblePoints[1].x - possiblePoints[1].y - slope * p1.x + p1.y) / Math.sqrt(slopeSquared + 1);

        double distanceFromLastPoint = MathFunctions.distance(allPoints.get(targetControlPoint-1).toPoint(), intersect);
        double distanceFromNextPoint = MathFunctions.distance(allPoints.get(targetControlPoint).toPoint(), intersect);
        double distanceMult = distanceFromLastPoint + distanceFromNextPoint;
        distanceMult = (distanceMult == 0) ? 0.5 : distanceFromLastPoint / distanceMult;
        double heading = allPoints.get(targetControlPoint-1).heading
                + (allPoints.get(targetControlPoint).heading - allPoints.get(targetControlPoint-1).heading) * distanceMult;

        ArrayList<Pose> pointsToAdd = new ArrayList<>();

        if (distance0 < distance1) {
            pointsToAdd.add(new Pose(possiblePoints[0], heading));
        } else {
            pointsToAdd.add(new Pose(possiblePoints[1], heading));
        }

        // the following is a derived formula, split into parts for efficiency
        double part4 = radius * Math.sqrt(slopeSquared + 1);

        double turningX0 = intersect.x + slopeSquared * intersect.x + part4;
        double turningX1 = intersect.x + slopeSquared * intersect.x - part4;
        Point[] turningPoints = {
                new Point(turningX0, slope * (turningX0 - intersect.x) + intersect.y),
                new Point(turningX1, slope * (turningX1 - intersect.x) + intersect.y)
        };

        pointsToAdd.add(0, new Pose(turningPoints[0], heading));
        pointsToAdd.add(new Pose(turningPoints[1], heading));

        allPoints.addAll(targetControlPoint, pointsToAdd);
    }

    public Point findIntersect(Point p1, Point p2) {
        // obtain intersects from occupancy map

        return new Point(1, 0);
    }
}