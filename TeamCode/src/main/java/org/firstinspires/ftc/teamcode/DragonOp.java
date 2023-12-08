package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        double move = -gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;
        double strafe = -gamepad1.left_stick_x;

        Actuation.drive(move, turn, strafe);

        if (gamepad2.dpad_left){
            Actuation.setExtension(ActuationConstants.Extension.extensionPresets[0]);
            Actuation.setTilt(ActuationConstants.Extension.tiltPresets[0]);
            Actuation.setTilt(ActuationConstants.Claw.wristDeposit);
        }
        else if (gamepad2.dpad_up){
            Actuation.setExtension(ActuationConstants.Extension.extensionPresets[1]);
            Actuation.setTilt(ActuationConstants.Extension.tiltPresets[1]);
            Actuation.setTilt(ActuationConstants.Claw.wristDeposit);
        }
        else if (gamepad2.dpad_right){
            Actuation.setExtension(ActuationConstants.Extension.extensionPresets[2]);
            Actuation.setTilt(ActuationConstants.Extension.tiltPresets[2]);
            Actuation.setTilt(ActuationConstants.Claw.wristDeposit);
        }
        else if (gamepad2.dpad_down){
            Actuation.setExtension(ActuationConstants.Extension.extensionPresets[3]);
            Actuation.setTilt(ActuationConstants.Extension.tiltPresets[3]);
            Actuation.setTilt(ActuationConstants.Claw.wristDeposit);
        }

        Actuation.toggleClaw(gamepad1.left_bumper);

        Actuation.toggleWrist(gamepad1.triangle);

        telemetry.addData("Extension Pos", Actuation.getExtension());
        telemetry.addData("move", move);
        telemetry.addData("turn", turn);
        telemetry.addData("strafe", strafe);
        telemetry.update();
    }
}
