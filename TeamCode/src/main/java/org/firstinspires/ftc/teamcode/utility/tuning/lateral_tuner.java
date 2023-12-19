package org.firstinspires.ftc.teamcode.utility.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.RobotMovement;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;

@Config
@TeleOp(name="Lateral Tuner", group="tuning")
public class lateral_tuner extends OpMode {
    RobotMovement robot;
    public double measured_dist = 0.0;

    ArrayList<Pose> traj;

    @Override
    public void init() {
        robot = new RobotMovement(hardwareMap, new Pose(0, 0, 0));

        traj = new ArrayList<>();
        traj.add(new Pose(0, 0, 0));
        traj.add(new Pose(48, 0, 0));
    }

    @Override
    public void loop() {
        robot.updatePosition();
        robot.incrementPoseCurve(traj, ActuationConstants.Autonomous.followDistance, ActuationConstants.Autonomous.moveSpeed, ActuationConstants.Autonomous.turnSpeed);

        telemetry.addData("X pos", robot.robotPose.x);
        telemetry.addData("new lateral multiplier: ", ActuationConstants.Drivetrain.lateral_multiplier * (measured_dist/robot.robotPose.x));
        telemetry.update();
    }
}
