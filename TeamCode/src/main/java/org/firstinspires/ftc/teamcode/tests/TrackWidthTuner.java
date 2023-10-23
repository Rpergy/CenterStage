package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(group = "Tests", name = "TrackWidthTuner")
@Config
public class TrackWidthTuner extends LinearOpMode {
    BHI260IMU imu;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

//    double leftOffset, rightOffset, backOffset;

    public static double wheel_circ, ticksPerRev, track_width, forward_offset;

    double scale;

    public static double lateral_offset;

    @Override
    public void runOpMode() {

        wheel_circ = 15.708; // cm
        track_width = 29.53/2; // cm distance between drive wheels
        forward_offset = -14.5; // cm distance from center of robot to perp wheel
        ticksPerRev = 8192;

        lateral_offset = 0.5;

        scale = wheel_circ / ticksPerRev;

        BHI260IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );

        imu = hardwareMap.get(BHI260IMU.class, "imu");//omkar is gay
        imu.initialize(parameters);
//peepeepoopoo lol
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

//        leftOffset = frontLeft.getCurrentPosition();
//        rightOffset = frontRight.getCurrentPosition();
//        backOffset = backRight.getCurrentPosition();

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        double turn = 1;
        frontLeft.setPower(turn);
        frontRight.setPower(turn);
        backLeft.setPower(turn);
        frontRight.setPower(turn);



        telemetry.addData("Status", "Initialized");
    }

}