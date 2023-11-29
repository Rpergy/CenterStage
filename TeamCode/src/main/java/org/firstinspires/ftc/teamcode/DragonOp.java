package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;

@TeleOp(name="Dragon Op")
public class DragonOp extends OpMode {
    @Override
    public void init() {
        Actuation.setup(hardwareMap);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        double move = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;

        Actuation.drive(move, turn, strafe);

        Actuation.setExtension((int)(gamepad1.left_trigger * 15));
        Actuation.setExtension((int)(-gamepad1.right_trigger * 15));

        Actuation.toggleClaw(gamepad1.left_bumper);
        Actuation.toggleTilt(gamepad1.triangle);
        Actuation.toggleWrist(gamepad1.triangle);
    }
}
