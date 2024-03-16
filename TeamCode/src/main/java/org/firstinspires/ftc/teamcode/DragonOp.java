package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.MathFunctions;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.RobotMovement;

@TeleOp(name="Dragon Op")
public class DragonOp extends OpMode {
    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        Actuation.setPlaneTilt(ActuationConstants.Plane.setupTilt);
    }

    @Override
    public void loop() {
        AutoMovement.updatePosition();
        AutoMovement.displayPosition();

        double move = gamepad1.left_stick_y;
        double turn = gamepad1.left_stick_x;
        double strafe = gamepad1.right_stick_x;

        if(gamepad1.left_bumper) {
            AutoMovement.turnTowards(0, 1.0);
        }
        else {
            Actuation.teleDrive(gamepad1.right_stick_button, gamepad1.left_stick_button, move, turn, strafe);
        }

        Actuation.toggleSlides(gamepad2.square); // slides

        // depositor
        if(gamepad2.left_trigger > 0.5 || gamepad1.left_trigger > 0.5 || gamepad1.right_trigger > 0.5) Actuation.toggleDeposit(1); // close
        else if(gamepad2.right_trigger > 0.5) Actuation.toggleDeposit(-1); // open
        else Actuation.toggleDeposit(0);

        // Ryan sucks at programming

        // pixel stack
        if(gamepad1.right_bumper) Actuation.setIntakeArm(ActuationConstants.Intake.stackPos[4]);
        else Actuation.setIntakeArm(ActuationConstants.Intake.stackPos[0]);

        // tilts
        if(gamepad2.left_bumper) Actuation.setTiltPreset(1);
        if(gamepad2.right_bumper) Actuation.setTiltPreset(2);
        if(gamepad2.triangle) Actuation.setTiltPreset(3);
        if(gamepad2.cross) Actuation.setTiltPreset(0);

        // hang controls
        if(gamepad2.dpad_up) {
            Actuation.setLeds(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
            Actuation.setTiltPreset(4);
        }
        if(gamepad2.share) Actuation.hangSetup();
        if(gamepad2.options) Actuation.hang();

        Actuation.autoDeposit();
        Actuation.stuckFix(gamepad2.ps);

        // intake
        if (gamepad1.left_trigger > 0.5) {
            Actuation.setIntake(-1.0);
        }
        else if (gamepad1.right_trigger > 0.5){
            Actuation.setIntake(1.0);
        }
        else {
            Actuation.setIntake(0.0);
        }

        // airplane
        if(gamepad2.dpad_left) Actuation.setPlaneTilt(ActuationConstants.Plane.launchTilt);
        if(gamepad2.dpad_right) Actuation.setPlaneLaunch(ActuationConstants.Plane.releaseUp);

        telemetry.addData("Slow mode", Actuation.slowMode);
        telemetry.addData("Field centric", Actuation.fieldCentric);
        telemetry.addData("x", AutoMovement.robotPose.x);
        telemetry.addData("y", AutoMovement.robotPose.y);
        telemetry.addData("heading", Math.toDegrees(MathFunctions.AngleWrap(AutoMovement.robotPose.heading)));
        telemetry.update();
    }
}
