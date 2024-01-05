package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.autonomous.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

public class TrajTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap);

        Trajectory traj = new Trajectory()
                .lineTo(new Pose(10, 0, Math.toRadians(90)))
                .lineTo(new Pose(10, 10, Math.toRadians(90)))
                .turnTo(0);
    }
}
