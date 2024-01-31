package org.firstinspires.ftc.teamcode.tests.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.autonomous.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@TeleOp(name = "Acceleration Test", group="tests")
public class AccelerationTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);

        Trajectory movement = new Trajectory(new Pose(0, 0, 0))
                .lineTo(new Pose(20, 0, 0))
                .lineTo(new Pose(20, 20, Math.toRadians(90)))
                .lineTo(new Pose(0, 20, Math.toRadians(270)))
                .lineTo(new Pose(0, 0, Math.toRadians(0)));

        waitForStart();
        while(opModeIsActive()) movement.run();
    }
}
