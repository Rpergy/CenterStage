package org.firstinspires.ftc.teamcode.tests.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.RobotMovement;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;

@Autonomous(group = "Test", name = "PurePursuit")
@Disabled
public class PurePursuit extends OpMode {
    RobotMovement robot;

    @Override
    public void init() {
        robot = new RobotMovement(hardwareMap, new Pose(0, 0, 0));
    }

    @Override
    public void loop() {
        robot.updatePosition();

        ArrayList<Pose> allPoses = new ArrayList<>();
        allPoses.add(new Pose(-50, 25, 0));
        allPoses.add(new Pose(0, 25, Math.toRadians(90)));
        allPoses.add(new Pose(0, -45, Math.toRadians(180)));

//        robot.goToPose(new Pose(0, 45, Math.toRadians(90)), ActuationConstants.Autonomous.moveSpeed, ActuationConstants.Autonomous.turnSpeed);

        robot.incrementPoseCurve(allPoses, ActuationConstants.Autonomous.followDistance, ActuationConstants.Autonomous.moveSpeed, ActuationConstants.Autonomous.turnSpeed);

        robot.displayPoses(allPoses, ActuationConstants.Autonomous.followDistance);
    }
}
