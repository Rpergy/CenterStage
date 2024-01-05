package org.firstinspires.ftc.teamcode.utility.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.RobotMovement;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;

@Config
@TeleOp(name="Perpendicular Tuner", group="tuning")
@Disabled
public class perpendicular_tuner extends OpMode {
    RobotMovement robot;
    public static double measured_dist = 0.0;

    ArrayList<Pose> traj;

    @Override
    public void init() {
        robot = new RobotMovement(hardwareMap, new Pose(0, 0, Math.toRadians(90)));

        traj = new ArrayList<>();
        traj.add(new Pose(0, 0, Math.toRadians(90)));
        traj.add(new Pose(48, 0, Math.toRadians(90)));
    }

    @Override
    public void loop() {
        robot.updatePosition();
        robot.incrementPoseCurve(traj, ActuationConstants.Autonomous.followDistance, ActuationConstants.Autonomous.moveSpeed, ActuationConstants.Autonomous.turnSpeed);
        robot.displayPoses(traj, ActuationConstants.Autonomous.followDistance);

        telemetry.addData("X pos", robot.robotPose.x);
        telemetry.addData("diff", measured_dist/robot.robotPose.x);
        telemetry.addData("new perpendicular multiplier", ActuationConstants.Drivetrain.perpendicular_multiplier * (measured_dist/robot.robotPose.x));
        telemetry.update();
    }
}
