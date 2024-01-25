package org.firstinspires.ftc.teamcode.tests.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;

@TeleOp(name="Dodge Test")
@Disabled
public class DodgeTest extends OpMode {
    DodgeMovement robot;

    @Override
    public void init() {
        robot = new DodgeMovement(hardwareMap, new Pose(11.5, 63, -90));
    }

    @Override
    public void loop() {
        robot.updatePosition();

        ArrayList<Pose> allPoses = new ArrayList<Pose>();
        allPoses.add(new Pose(11.5, 0, -90));

        robot.incrementPoseCurve(allPoses, ActuationConstants.Autonomous.followDistance, ActuationConstants.Autonomous.moveSpeed, ActuationConstants.Autonomous.turnSpeed);
        robot.displayPoses(allPoses, ActuationConstants.Autonomous.followDistance);
    }
}