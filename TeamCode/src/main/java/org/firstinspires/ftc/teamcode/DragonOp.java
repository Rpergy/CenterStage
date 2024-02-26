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

        Actuation.teleDrive(gamepad1.right_stick_button, gamepad1.left_stick_button, move, turn, strafe);

        Actuation.toggleSlides(gamepad2.square);
        Actuation.toggleTilt(gamepad2.cross);

        Actuation.toggleDeposit(gamepad2.circle || gamepad1.circle);

        if(gamepad2.dpad_up) Actuation.setTilt(ActuationConstants.Extension.tiltPositions[2]);
        if(gamepad2.share) Actuation.hangSetup();
        if(gamepad2.options) Actuation.hang();

        Actuation.autoDeposit();

        if (gamepad1.left_trigger > 0.5)
            Actuation.setIntake(-1.0);
        else if (gamepad1.right_trigger > 0.5){
            Actuation.setIntake(1.0);
        }
        else {
            Actuation.setIntake(0.0);
        }

        telemetry.addData("Slow mode", Actuation.slowMode);
        telemetry.addData("Field centric", Actuation.fieldCentric);
        telemetry.addData("x", AutoMovement.robotPose.x);
        telemetry.addData("y", AutoMovement.robotPose.y);
        telemetry.addData("heading", Math.toDegrees(MathFunctions.AngleWrap(AutoMovement.robotPose.heading)));
        telemetry.update();
    }
}
