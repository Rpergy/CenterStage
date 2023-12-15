package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;

@TeleOp(name="Dragon Op")
public class DragonOp extends OpMode {
    private int currentTilt = 3;
    private boolean slowModeToggle = false;
    private boolean slowMode = false;
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

        if (slowMode)
            Actuation.drive(move*0.5, turn*0.5, strafe*0.5);
        else
            Actuation.drive(move, turn, strafe);

        if(gamepad1.right_stick_button && !slowModeToggle) {
            slowMode = !slowMode;
            slowModeToggle = true;
        }
        else if(!gamepad1.right_stick_button)
            slowModeToggle = false;

        if (gamepad2.dpad_left){
            Actuation.setTilt(ActuationConstants.Extension.tiltPresets[0]);
            currentTilt = 0;
        }
        else if (gamepad2.dpad_up){
            Actuation.setTilt(ActuationConstants.Extension.tiltPresets[1]);
            currentTilt = 1;
        }
        else if (gamepad2.dpad_right){
            Actuation.setTilt(ActuationConstants.Extension.tiltPresets[2]);
            currentTilt = 2;
        }
        else if (gamepad2.dpad_down){
            currentTilt = 3;
            Actuation.setTilt(ActuationConstants.Extension.tiltPresets[3]);
            Actuation.setExtension(ActuationConstants.Extension.extensionPresets[3]);
            Actuation.setWrist(ActuationConstants.Claw.wristIntake);
        }

        if (gamepad2.square) {
            if (currentTilt != 3) {
                Actuation.setExtension(ActuationConstants.Extension.extensionPresets[currentTilt]);
                Actuation.setWrist(ActuationConstants.Claw.wristDeposit);
            }
        }

        Actuation.toggleClaw(gamepad1.left_bumper);
        if (Actuation.getClawState()) {
            gamepad1.setLedColor(0, 1.0, 0, 200);
            gamepad2.setLedColor(0, 1.0, 0, 200);
        }
        else {
            gamepad1.setLedColor(1.0, 0.0, 0.0, 200);
            gamepad2.setLedColor(1.0, 0.0, 0, 200);
        }

        Actuation.toggleWrist(gamepad1.triangle);

        telemetry.addData("Extension Pos", Actuation.getExtension());
        telemetry.addData("Slow mode", slowMode);
        telemetry.addData("claw", Actuation.getClawState());
        telemetry.update();
    }
}
