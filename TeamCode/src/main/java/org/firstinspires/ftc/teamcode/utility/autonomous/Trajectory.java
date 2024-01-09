package org.firstinspires.ftc.teamcode.utility.autonomous;

import static org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement.robotPose;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.MathFunctions;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

public class Trajectory {
    double moveSpeed, turnSpeed, viewDist;

    public Trajectory() {
        moveSpeed = ActuationConstants.Autonomous.moveSpeed;
        turnSpeed = ActuationConstants.Autonomous.turnSpeed;
        viewDist = ActuationConstants.Autonomous.followDistance;
    }

    public Trajectory(Pose startPos) {
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

        return this;
    }

    /**
     * Moves the robot to a specified pose at a specific speed
     * @param targetPose Robot's target pose
     * @param mSpeed Robot's defined move speed
     * @return parent trajectory
     */
    public Trajectory lineTo(Pose targetPose, double mSpeed) {
        double dist = MathFunctions.distance(robotPose.toPoint(), targetPose.toPoint());
        double rotDist = robotPose.heading - targetPose.heading;

        while(dist > 0.6 || Math.abs(MathFunctions.AngleWrap(rotDist)) > Math.toRadians(4)) {
            AutoMovement.updatePosition();
            AutoMovement.moveTowards(targetPose, mSpeed, turnSpeed);

            dist = MathFunctions.distance(robotPose.toPoint(), targetPose.toPoint());
            rotDist = Math.abs(robotPose.heading - targetPose.heading);

            AutoMovement.telemetry.addData("dist", dist);
            AutoMovement.telemetry.addData("rotDist", Math.toDegrees(MathFunctions.AngleWrap(rotDist)));
            AutoMovement.telemetry.update();
        }

        Actuation.drive(0, 0, 0);

        return this;
    }

    public Trajectory turnTo(double targetHeading) {
        double rotDist = Math.abs(MathFunctions.AngleWrap(robotPose.heading - targetHeading));
        while(rotDist > Math.toRadians(2)) {
            AutoMovement.updatePosition();
            AutoMovement.turnTowards(targetHeading, turnSpeed);

            rotDist = Math.abs(robotPose.heading - targetHeading);
        }

        Actuation.drive(0, 0, 0);

        return this;
    }

    public Trajectory action(Runnable action)  {
        action.run();

        return this;
    }
}
