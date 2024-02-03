package org.firstinspires.ftc.teamcode.utility.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.RobotMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;

@Config
@TeleOp(name="Center Tuner", group="tuning")
public class centerTuner extends OpMode {
    public static double measuredDist = 0.0;
    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        AutoMovement.updatePosition();

        Actuation.drive(gamepad1.left_stick_y, 0.0, 0.0);

        telemetry.addData("X pos", AutoMovement.robotPose.x);
        telemetry.addData("diff", measuredDist /AutoMovement.robotPose.x);
        telemetry.addData("new center multiplier: ", ActuationConstants.Drivetrain.centerMultiplier * (measuredDist /AutoMovement.robotPose.x));
        telemetry.update();
    }
}
