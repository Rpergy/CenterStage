package org.firstinspires.ftc.teamcode.utility.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.RobotMovement;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;

@Config
@TeleOp(name="Perpendicular Tuner", group="tuning")
public class perpendicularTuner extends OpMode {
    public static double measuredDist = 0.0;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        AutoMovement.updatePosition();

        Actuation.drive(0, 0, gamepad1.left_stick_x);

        telemetry.addData("Y pos", AutoMovement.robotPose.y);
        telemetry.addData("diff", measuredDist / AutoMovement.robotPose.y);
        telemetry.addData("new perpendicular multiplier", ActuationConstants.Drivetrain.perpendicularMultiplier * (measuredDist /AutoMovement.robotPose.y));
        telemetry.update();
    }
}
