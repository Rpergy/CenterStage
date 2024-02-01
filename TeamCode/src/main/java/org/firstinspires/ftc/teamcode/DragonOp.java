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
    private int currentTilt = 3;

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
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;

//        if (gamepad2.dpad_left){
//            Actuation.setTilt(ActuationConstants.Extension.tiltPresets[0]);
//            currentTilt = 0;
//        }
//        else if (gamepad2.dpad_up){
//            Actuation.setTilt(ActuationConstants.Extension.tiltPresets[1]);
//            currentTilt = 1;
//        }
//        else if (gamepad2.dpad_right){
//            Actuation.setTilt(ActuationConstants.Extension.tiltPresets[2]);
//            currentTilt = 2;
//        }
//        else if (gamepad2.dpad_down){
//            currentTilt = 3;
//            Actuation.setTilt(ActuationConstants.Extension.tiltPresets[3]);
//            Actuation.setExtension(ActuationConstants.Extension.extensionPresets[3]);
//            Actuation.setWrist(ActuationConstants.Claw.wristIntake);
//        }
//
//        if (gamepad2.square) {
//            if (currentTilt != 3) {
//                Actuation.setExtension(ActuationConstants.Extension.extensionPresets[currentTilt]);
//                Actuation.setWrist(ActuationConstants.Claw.wristDeposit);
//            }
//        }
//
//        if (Actuation.getClawState()) {
//            gamepad1.setLedColor(0, 1.0, 0, 200);
//            gamepad2.setLedColor(0, 1.0, 0, 200);
//        }
//        else {
//            gamepad1.setLedColor(1.0, 0.0, 0.0, 200);
//            gamepad2.setLedColor(1.0, 0.0, 0, 200);
//        }
//
//        Actuation.toggleClaw(gamepad1.left_bumper);
//        Actuation.toggleWrist(gamepad1.triangle);
        Actuation.teleDrive(gamepad1.right_stick_button, gamepad1.left_stick_button, move, strafe, turn);
//
//        Actuation.setColors();

        telemetry.addData("Slow mode", Actuation.slowMode);
        telemetry.addData("Field centric", Actuation.fieldCentric);
        telemetry.addData("x", AutoMovement.robotPose.x);
        telemetry.addData("y", AutoMovement.robotPose.y);
        telemetry.addData("heading", Math.toDegrees(MathFunctions.AngleWrap(AutoMovement.robotPose.heading)));
        telemetry.update();
    }
}
