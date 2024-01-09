package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@Autonomous(name = "")
public class TrajTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);

        waitForStart();
        Trajectory traj = new Trajectory()
            .lineTo(new Pose(25, 0, Math.toRadians(90)))
            .lineTo(new Pose(25, 25, Math.toRadians(90)))
            .action(() -> Actuation.setWrist(ActuationConstants.Claw.wristIntake))
            .turnTo(0);

    }
}
