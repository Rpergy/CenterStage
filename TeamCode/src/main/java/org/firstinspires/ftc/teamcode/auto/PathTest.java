package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.RobotMovement;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;
import java.util.ArrayList;

@TeleOp(name="path test", group="tests")
public class PathTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotMovement robot = new RobotMovement(hardwareMap, new Pose(11.5, 63, Math.toRadians(-90)));

        ArrayList<Pose> traj = new ArrayList<>();
        traj.add(new Pose(11.5, 63, Math.toRadians(-90)));
        traj.add(new Pose(10, 50, Math.toRadians(-100)));

        waitForStart();
        while (opModeIsActive()) {
            robot.displayPoses(traj, ActuationConstants.Autonomous.followDistance);
        }
    }
}
