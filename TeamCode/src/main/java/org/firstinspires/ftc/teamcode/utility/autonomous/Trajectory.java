package org.firstinspires.ftc.teamcode.utility.autonomous;

import static org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement.robotPose;

import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.MathFunctions;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

public class Trajectory {
    double moveSpeed, turnSpeed, viewDist;

    public Trajectory() {
        moveSpeed = ActuationConstants.Autonomous.moveSpeed;
        turnSpeed = ActuationConstants.Autonomous.turnSpeed;
        viewDist = ActuationConstants.Autonomous.followDistance;
    }

    /**
     * Move the robot to another pose
     * @param targetPose The robot's targeted pose
     * @return Parent trajectory
     */
    public Trajectory lineTo(Pose targetPose) {
        double dist = MathFunctions.distance(robotPose.toPoint(), targetPose.toPoint());
        double rotDist = Math.abs(robotPose.heading - targetPose.heading);

        while(dist > 0.45 || rotDist > Math.toRadians(2)) {
            AutoMovement.updatePosition();
            AutoMovement.moveTowards(targetPose, moveSpeed, turnSpeed);

            dist = MathFunctions.distance(robotPose.toPoint(), targetPose.toPoint());
            rotDist = Math.abs(robotPose.heading - targetPose.heading);
        }

        return this;
    }

    public Trajectory turnTo(double targetHeading) {
        double rotDist = Math.abs(robotPose.heading - targetHeading);
        while(rotDist > Math.toRadians(2)) {
            AutoMovement.updatePosition();
            AutoMovement.turnTowards(targetHeading, turnSpeed);

            rotDist = Math.abs(robotPose.heading - targetHeading);
        }
        return this;
    }
}
