package org.firstinspires.ftc.teamcode.utility.tests.purepursuit;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.RobotMovement;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;

@TeleOp(name="Waypoint Drive")
public class WaypointDrive extends OpMode {
    RobotMovement robot;

    @Override
    public void init() {
        Actuation.setup(hardwareMap);
        robot = new RobotMovement(hardwareMap, new Pose(50, -50, 0));

//        toBlueCanvas = new ArrayList<>();
//        toBlueCanvas.add(new Pose(50, -50, Math.toRadians(0)));
//        toBlueCanvas.add(new Pose(0, -35, Math.toRadians(0)));
//        toBlueCanvas.add(new Pose(0, 35, Math.toRadians(90)));
//        toBlueCanvas.add(new Pose(-40, 55, Math.toRadians(90)));
    }

    @Override
    public void loop() {
        robot.updatePosition();

//        if (gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x >= 0.1)
//            Actuation.drive(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
//        else
//            robot.followPoseCurve(toBlueCanvas, ActuationConstants.Autonomous.followDistance, ActuationConstants.Autonomous.moveSpeed, ActuationConstants.Autonomous.turnSpeed);

//        robot.displayPoses(toBlueCanvas, ActuationConstants.Autonomous.followDistance);
    }
}
