package org.firstinspires.ftc.teamcode.utility.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.MathFunctions;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;
import java.util.List;

public class AutoMovement {
    public static Pose robotPose;

    static double dx, dy, dtheta;
    static double dx_center, dx_perpendicular;
    static double prev_ticks_left, prev_ticks_right, prev_ticks_back;

    static double searchIncrease = 1;

    static int targetControlPoint = 0;
    static int side_length = 6;

    static boolean lockOnEnd = false;

    static List<LynxModule> allHubs;
    static VoltageSensor voltageSensor;

    static FtcDashboard dashboard;

    public static Telemetry telemetry;

    /**
     * Necessary in order to correctly initialize reading from odometry and accessing voltage sensor
     * @param hardwareMap Current hardware map
     */
    public static void setup(HardwareMap hardwareMap, Telemetry tel) {
        robotPose = new Pose(0, 0, 0);

        telemetry = tel;

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("turnPower", 0);
        packet.put("movePower", 0);
        packet.put("strafePower", 0);
        dashboard.sendTelemetryPacket(packet);
    }

    /**
     * Updates the robot's pose based off of encoder values from odometry
     */
    public static void updatePosition() {
        TelemetryPacket packet = new TelemetryPacket();
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        double ticks_left = Actuation.frontLeft.getCurrentPosition();
        double ticks_right = Actuation.frontRight.getCurrentPosition();
        double ticks_back = Actuation.backRight.getCurrentPosition();

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
    }

    /**
     * Sets motor powers to move in the direction of a point. Must be called iteratively in order to control velocity along the line.
     * @param targetPose Robot's target pose
     * @param movementSpeed Robot's move speed
     * @param turnSpeed Robot's turn speed
     */
    public static void moveTowards(Pose targetPose, double movementSpeed, double turnSpeed) {
        double deltaX = targetPose.x - robotPose.x;
        double deltaY = targetPose.y - robotPose.y;

        double deltaTheta = MathFunctions.AngleWrap(targetPose.heading - robotPose.heading);

        double turnPower = deltaTheta/Math.PI * turnSpeed;

        double m1 = (Math.tanh(deltaY * ActuationConstants.Autonomous.moveAccelMult) * Math.sin(robotPose.heading)) * movementSpeed;
        double m2 = (Math.tanh(deltaX * ActuationConstants.Autonomous.moveAccelMult) * Math.cos(robotPose.heading)) * movementSpeed;

        double s1 = (-Math.tanh(deltaY * ActuationConstants.Autonomous.moveAccelMult) * Math.cos(robotPose.heading)) * movementSpeed;
        double s2 = (Math.tanh(deltaX * ActuationConstants.Autonomous.moveAccelMult) * Math.sin(robotPose.heading)) * movementSpeed;

        double movePower = (m1 * Math.abs(m1) + m2 * Math.abs(m2));
        double strafePower =  (s1 * Math.abs(s1) + s2 * Math.abs(s2));

        if(turnPower > 0) turnPower = Math.pow(turnPower, 1.0/ActuationConstants.Autonomous.turnAccelMult) * turnSpeed;
        else turnPower = -Math.pow(-turnPower, 1.0/ActuationConstants.Autonomous.turnAccelMult) * turnSpeed;

//        if(movePower > 0) movePower = Math.max(movePower, ActuationConstants.Autonomous.minMoveSpeed);
//        else movePower = Math.min(movePower, -ActuationConstants.Autonomous.minMoveSpeed);

        double v1 = -movePower + turnPower - strafePower;
        double v2 = -movePower - turnPower + strafePower;
        double v3 = -movePower + turnPower + strafePower;
        double v4 = -movePower - turnPower - strafePower;

        double voltageComp = 12 / voltageSensor.getVoltage();

        Actuation.frontLeft.setPower(v1 * voltageComp);
        Actuation.frontRight.setPower(v2 * voltageComp);
        Actuation.backLeft.setPower(v3 * voltageComp);
        Actuation.backRight.setPower(v4 * voltageComp);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("movePower", movePower);
        packet.put("turnPower", turnPower);
        packet.put("strafePower", strafePower);
        dashboard.sendTelemetryPacket(packet);
    }

    public static void turnTowards(double targetHeading, double turnSpeed) {
        double deltaTheta = MathFunctions.AngleWrap(targetHeading - robotPose.heading);
        double turnPower = deltaTheta/Math.PI;

        if(turnPower > 0) {
            turnPower = Math.pow(turnPower, 1.0/ActuationConstants.Autonomous.turnAccelMult) * turnSpeed;
        }
        else {
            turnPower = -Math.pow(-turnPower, 1.0/ActuationConstants.Autonomous.turnAccelMult) * turnSpeed;
        }

//        if (deltaTheta > 0)
//            turnPower = Math.max(turnPower, ActuationConstants.Autonomous.minTurnSpeed);
//        else
//            turnPower = Math.min(turnPower, -ActuationConstants.Autonomous.minTurnSpeed);

        double voltageComp = 12 / voltageSensor.getVoltage();

        Actuation.frontLeft.setPower(turnPower * voltageComp);
        Actuation.frontRight.setPower(-turnPower * voltageComp);
        Actuation.backLeft.setPower(turnPower * voltageComp);
        Actuation.backRight.setPower(-turnPower * voltageComp);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("turnPower", turnPower);
        dashboard.sendTelemetryPacket(packet);
    }

    /**
     * Finds the current pose that the robot should be following based off of intersections of ControlLines and the view circle
     * @param pathPoints List including all poses for the lines in the curve
     * @param followRadius Intersections search distance
     * @return Interpolated pose which the robot should move towards
     */
    public static Pose getFollowPose(ArrayList<Pose> pathPoints, double followRadius) {
        Pose followMe = new Pose(pathPoints.get(0));

        ArrayList<Pose> intersections = new ArrayList<>();

//      iteratively increasing followRadius until valid line circle intersection is found(?)
        while (intersections.size() == 0) {
            for (int i = 0; i < pathPoints.size() - 1; i++) {
                Pose startLine = pathPoints.get(i);
                Pose endLine = pathPoints.get(i + 1);
                intersections.addAll(MathFunctions.lineCircleIntersection(robotPose.toPoint(), followRadius, startLine, endLine));
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
     * Sets the robot off in a direction following a path defined by poses. Must be called iteratively in order to account for curves in the path.
     * @param allPoints List including all poses for the lines in the curve
     * @param followDistance Distance specifying how closely the robot follows the defined curve
     * @param moveSpeed Motor power multiplier for movement motion
     * @param turnSpeed Motor power multiplier for turning motion
     */
    public static void incrementPoseCurve(ArrayList<Pose> allPoints, double followDistance, double moveSpeed, double turnSpeed) {
        double distanceToTarget = MathFunctions.distance(robotPose.toPoint(), allPoints.get(targetControlPoint).toPoint());

//        only move along the point list if we are closer to the target than the follow distance
//        and if targetControlPoint is less than the last point in the list
//        targetControlPoint is an index into the list
//        since it starts at zero this condition is just stopping incrementing the pose curve list
//        when we reach the end of it so we don't get an out of bounds exception
        if (distanceToTarget <= followDistance && targetControlPoint < allPoints.size()-1) {
            targetControlPoint++;
        }


        Pose followMe = getFollowPose(allPoints, followDistance);
        if (followMe.withinRange(allPoints.get(allPoints.size()-1), 1.0) || lockOnEnd && targetControlPoint == allPoints.size()-1) {
            moveTowards(allPoints.get(allPoints.size()-1), moveSpeed, turnSpeed);
            lockOnEnd = true;
        }
        else {
            moveTowards(followMe, moveSpeed, turnSpeed);
        }
    }

    /**
     * Shows a path of points as well as the robot's position on the FTC Dashboard
     * @param allPoints List of points to show as a path
     */
    public static void displayPoses(ArrayList<Pose> allPoints, double radius) {
        TelemetryPacket packet = new TelemetryPacket();

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

        Pose followMe = getFollowPose(allPoints, radius);
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

        dashboard.sendTelemetryPacket(packet);
    }

    /**
     * Displays the robot's position on the FTC dashboard
     */
    public static void displayPosition(){
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
}
