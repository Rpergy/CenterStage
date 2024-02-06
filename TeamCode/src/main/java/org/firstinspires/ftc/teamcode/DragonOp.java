package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.MathFunctions;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.RobotMovement;

@TeleOp(name="Dragon Op")
public class DragonOp extends OpMode {

    private RobotMovement robot;
    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        AutoMovement.updatePosition();
        AutoMovement.displayPosition();

        double move = gamepad1.left_stick_y;
        double turn = gamepad1.left_stick_x;
        double strafe = gamepad1.right_stick_x;

        Actuation.teleDrive(gamepad1.right_stick_button, gamepad1.left_stick_button, move, strafe, turn);

        telemetry.addData("Slow mode", Actuation.slowMode);
        telemetry.addData("Field centric", Actuation.fieldCentric);
        telemetry.addData("x", AutoMovement.robotPose.x);
        telemetry.addData("y", AutoMovement.robotPose.y);
        telemetry.addData("heading", Math.toDegrees(MathFunctions.AngleWrap(AutoMovement.robotPose.heading)));
        telemetry.update();
    }
}
