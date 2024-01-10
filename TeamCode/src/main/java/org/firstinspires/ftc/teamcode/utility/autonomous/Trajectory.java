package org.firstinspires.ftc.teamcode.utility.autonomous;

import static org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement.robotPose;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.MathFunctions;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;

public class Trajectory {
    double moveSpeed, turnSpeed, viewDist;

    ArrayList<Runnable> movements;

    public Trajectory() {
        movements = new ArrayList<>();

        moveSpeed = ActuationConstants.Autonomous.moveSpeed;
        turnSpeed = ActuationConstants.Autonomous.turnSpeed;
        viewDist = ActuationConstants.Autonomous.followDistance;
    }

    public Trajectory(Pose startPos) {
        movements = new ArrayList<>();

        moveSpeed = ActuationConstants.Autonomous.moveSpeed;
        turnSpeed = ActuationConstants.Autonomous.turnSpeed;
        viewDist = ActuationConstants.Autonomous.followDistance;
        robotPose = startPos;
    }

    /**
     * Move the robot to another pose
     * @param targetPose The robot's targeted pose
     * @return Parent trajectory
     */
    public Trajectory lineTo(Pose targetPose) {
        movements.add(() -> runLineTo(targetPose));
        return this;
    }

    /**
     * Move the robot to another pose with a specific speed
     * @param targetPose The robot's targeted pose
     * @param moveSpeed The robot's target move speed
     * @param turnSpeed The robot's target turn speed
     * @return Parent trajectory
     */
    public Trajectory lineTo(Pose targetPose, double moveSpeed, double turnSpeed) {
        movements.add(() -> runLineTo(targetPose, moveSpeed, turnSpeed));
        return this;
    }

    /**
     * Turn the robot to a target heading
     * @param target Robot's target heading
     * @return Parent trajectory
     */
    public Trajectory turnTo(double target) {
        movements.add(() -> runTurnTo(target));
        return this;
    }

    /**
     * Specifies an action for the robot to carry out
     * @param action Runnable robot action
     * @return Parent trajectory
     */
    public Trajectory action(Runnable action) {
        movements.add(action);
        return this;
    }

    /**
     * Runs the trajectory's previously specified movements
     */
    public void run() {
        for (Runnable movement : movements) {
            movement.run();
        }
    }

    private void runLineTo(Pose targetPose) {
        double dist = MathFunctions.distance(robotPose.toPoint(), targetPose.toPoint());
        double rotDist = robotPose.heading - targetPose.heading;

        while(dist > 0.45 || Math.abs(MathFunctions.AngleWrap(rotDist)) > Math.toRadians(4)) {
            AutoMovement.updatePosition();
            AutoMovement.moveTowards(targetPose, moveSpeed, turnSpeed);

            dist = MathFunctions.distance(robotPose.toPoint(), targetPose.toPoint());
            rotDist = Math.abs(robotPose.heading - targetPose.heading);

            AutoMovement.telemetry.addData("dist", dist);
            AutoMovement.telemetry.addData("rotDist", Math.toDegrees(MathFunctions.AngleWrap(rotDist)));
            AutoMovement.telemetry.update();
        }

        Actuation.drive(0, 0, 0);
    }

    private void runLineTo(Pose targetPose, double mSpeed, double tSpeed) {
        double dist = MathFunctions.distance(robotPose.toPoint(), targetPose.toPoint());
        double rotDist = robotPose.heading - targetPose.heading;

        while(dist > 0.6 || Math.abs(MathFunctions.AngleWrap(rotDist)) > Math.toRadians(4)) {
            AutoMovement.updatePosition();
            AutoMovement.moveTowards(targetPose, mSpeed, tSpeed);

            dist = MathFunctions.distance(robotPose.toPoint(), targetPose.toPoint());
            rotDist = Math.abs(robotPose.heading - targetPose.heading);

            AutoMovement.telemetry.addData("dist", dist);
            AutoMovement.telemetry.addData("rotDist", Math.toDegrees(MathFunctions.AngleWrap(rotDist)));
            AutoMovement.telemetry.update();
        }

        Actuation.drive(0, 0, 0);
    }

    private void runTurnTo(double targetHeading) {
        double rotDist = Math.abs(MathFunctions.AngleWrap(robotPose.heading - targetHeading));
        while(rotDist > Math.toRadians(2)) {
            AutoMovement.updatePosition();
            AutoMovement.turnTowards(targetHeading, turnSpeed);

            rotDist = Math.abs(robotPose.heading - targetHeading);
        }

        Actuation.drive(0, 0, 0);
    }
}
