package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.ActuationConstants;

@TeleOp(name="Dragon Op")
public class DragonOp extends OpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private Servo arm;
    private Servo wrist;
    private Servo leftClaw;
    private Servo rightClaw;

    DcMotor extension;

    @Override
    public void init() {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm = hardwareMap.servo.get("extensionTilt");
        arm.setPosition(ActuationConstants.Extension.tiltIntake);

        wrist = hardwareMap.servo.get("clawWrist");
        wrist.setPosition(ActuationConstants.Claw.wristIntake);

        leftClaw = hardwareMap.servo.get("leftClaw");
        leftClaw.setPosition(ActuationConstants.Claw.open);

        rightClaw = hardwareMap.servo.get("rightClaw");
        rightClaw.setDirection(Servo.Direction.REVERSE);
        rightClaw.setPosition(ActuationConstants.Claw.open);

        extension = hardwareMap.dcMotor.get("extension");
        extension.setPower(1.0);
        extension.setTargetPosition(ActuationConstants.Extension.extensionStart);
        extension.setDirection(DcMotorSimple.Direction.REVERSE);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        double move = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;

        frontLeft.setPower(move - turn - strafe);
        frontRight.setPower(move + turn + strafe);
        backLeft.setPower(move - turn + strafe);
        backRight.setPower(move + turn - strafe);

        boolean flip = gamepad1.right_trigger > 0.85; // both wrist and arm
        boolean openCloseClaw = gamepad1.left_bumper;

        if (gamepad1.dpad_up) {
            extension.setTargetPosition(extension.getCurrentPosition() + 60);
        }
        else if (gamepad1.dpad_down) {
            extension.setTargetPosition(extension.getCurrentPosition() - 60);
        }

        if (flip) {
            arm.setPosition(ActuationConstants.Extension.tiltDeposit);
            wrist.setPosition(ActuationConstants.Claw.wristDeposit);
        } else {
            arm.setPosition(ActuationConstants.Extension.tiltIntake);
            wrist.setPosition(ActuationConstants.Claw.wristIntake);
        }

        if (openCloseClaw) {
            leftClaw.setPosition(ActuationConstants.Claw.closed);
            rightClaw.setPosition(ActuationConstants.Claw.closed);
        } else {
            leftClaw.setPosition(ActuationConstants.Claw.open);
            rightClaw.setPosition(ActuationConstants.Claw.open);
        }
    }
}
