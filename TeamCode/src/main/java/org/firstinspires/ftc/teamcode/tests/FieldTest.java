package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@TeleOp(name = "FieldTest", group = "tests")
@Config
public class FieldTest extends OpMode {
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private IMU imu;
    private int count = 0;
    double startTime = 0, endTime, timeDif;
    private double heading;

    FtcDashboard dashboard;
    @Override
    public void init() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        )));

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");

        dashboard = FtcDashboard.getInstance();
    }
    @Override
    public void loop() {
        //startTime = System.nanoTime();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        heading = orientation.getYaw(AngleUnit.RADIANS);
        double move = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;
        double newMove = strafe*Math.sin(-heading)+move*Math.cos(-heading);
        double newStrafe = strafe*Math.cos(-heading)-move*Math.sin(-heading);

        frontLeft.setPower(newMove+newStrafe+turn);
        backLeft.setPower(newMove-newStrafe+turn);
        frontRight.setPower(newMove-newStrafe-turn);
        backRight.setPower(newMove+newStrafe-turn);

        if (gamepad1.y) {
            telemetry.addData("Yaw", "Resetting\n");
            double original = heading;
            imu.resetYaw();
            if (orientation.getYaw(AngleUnit.RADIANS) != original)
                startTime=System.nanoTime();
        } else {
            telemetry.addData("Yaw", "Press Y (triangle) on Gamepad to reset\n");
        }
        endTime = System.nanoTime();
        timeDif = endTime-startTime;
        if (Math.abs(heading) < 0.01 && timeDif > 5000) {
            count++;
        }
        if (count > 100) {
            gamepad1.rumble(2500);
            count=0;
        }
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        //telemetry.addData("Time", System.nanoTime());
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("yaw", orientation.getYaw(AngleUnit.DEGREES));
        dashboard.sendTelemetryPacket(packet);
    }
}
