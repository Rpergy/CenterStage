package org.firstinspires.ftc.teamcode.utility.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.RobotMovement;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;

@Autonomous(name = "Drift Test")
//@Disabled
public class DriftTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap);
        Actuation.setClaw(ActuationConstants.Claw.closed);
        Actuation.setWrist(ActuationConstants.Claw.wristAutoInit);
        Actuation.setTilt(0.3);

        RobotMovement robot = new RobotMovement(hardwareMap, new Pose(0, 0, Math.toRadians(0)));

        ArrayList<Pose> testTraj = new ArrayList<>();
        testTraj.add(new Pose(0, 0, 0));

//        for (int i = 0; i < 10; i++) {
        testTraj.add(new Pose(10, 0, Math.toRadians(0)));
        testTraj.add(new Pose(20, 0, Math.toRadians(0)));
        testTraj.add(new Pose(30, 0, Math.toRadians(0)));
//            testTraj.add(new Pose(40, 40, Math.toRadians(0)));
//            testTraj.add(new Pose(0, 40, Math.toRadians(0)));
//            testTraj.add(new Pose(0, 0, 0));
//        }

        waitForStart();
        robot.followPoseCurve(telemetry, testTraj, ActuationConstants.Autonomous.followDistance, ActuationConstants.Autonomous.moveSpeed, ActuationConstants.Autonomous.turnSpeed);
    }
}
