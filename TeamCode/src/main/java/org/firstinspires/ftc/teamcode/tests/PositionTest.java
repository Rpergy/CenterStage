package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(group = "Tests", name = "Position Test")
@Config
public class PositionTest extends OpMode {
    BHI260IMU imu;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

//    double leftOffset, rightOffset, backOffset;

    public static double wheel_circ, ticksPerRev, track_width, forward_offset;
    double fwd, str;

    double x, y, theta;

    double prev_ticks_left = 0, prev_ticks_right = 0, prev_ticks_back = 0;

    double dx, dy, dtheta;

    double dx_center, dx_perpendicular;

    double side_length = 5;

    double scale = 0.0737;

    @Override
    public void init() {

        wheel_circ = 157.08; // mm
        track_width = 280; // mm distance between drive wheels
        forward_offset = 145; // mm distance from center of robot to perp wheel
        ticksPerRev = 8192;

        BHI260IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );

        imu = hardwareMap.get(BHI260IMU.class, "imu");//omkar is gay
        imu.initialize(parameters);

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

//        leftOffset = frontLeft.getCurrentPosition();
//        rightOffset = frontRight.getCurrentPosition();
//        backOffset = backRight.getCurrentPosition();

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {

        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        double[] xs = {(side_length * Math.cos(theta) - side_length * Math.sin(theta)) + 5,
                (-side_length * Math.cos(theta) - side_length * Math.sin(theta)) + 5,
                (side_length * Math.sin(theta) - side_length * Math.cos(theta)) + 5,
                (side_length * Math.cos(theta) + side_length * Math.sin(theta)) + 5};

//        double[] xs = {scale * x + side_length, scale * x + side_length, scale * x + -side_length, scale * x + -side_length};

//        double[] ys = {scale * y + side_length, scale * y + -side_length, scale * y + -side_length, scale * y + side_length};

        double[] ys = {(side_length * Math.sin(theta) + side_length * Math.cos(theta)) + 5,
                (-side_length * Math.sin(theta) + side_length * Math.cos(theta)) + 5,
                (-side_length * Math.cos(theta) - side_length * Math.sin(theta)) + 5,
                (side_length * Math.sin(theta) - side_length * Math.cos(theta)) + 5};

        packet.fieldOverlay().fillPolygon(xs, ys).setFill("blue");

        double move = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;

        frontLeft.setPower(move - turn + strafe);
        frontRight.setPower(move + turn - strafe);
        backLeft.setPower(move - turn - strafe);
        backRight.setPower(move + turn + strafe);

        double delta_ticks_left = (frontLeft.getCurrentPosition() - prev_ticks_left) / ticksPerRev * wheel_circ;
        double delta_ticks_right = (frontRight.getCurrentPosition() - prev_ticks_right) / ticksPerRev * wheel_circ;
        double delta_ticks_back = (backRight.getCurrentPosition() - prev_ticks_back) / ticksPerRev * wheel_circ;

        dtheta = ((delta_ticks_left - delta_ticks_right) / track_width) * 2;
        dx_center = (delta_ticks_left + delta_ticks_right) / 2;
        dx_perpendicular = delta_ticks_back - (forward_offset * dtheta);

        dx = dx_center * Math.cos(theta) - dx_perpendicular * Math.sin(theta);
        dy = dx_center * Math.sin(theta) + dx_perpendicular * Math.cos(theta);

        x += dx;
        y += dy;
        theta += dtheta;


        telemetry.addData("ticks back", prev_ticks_back);
        telemetry.addData("ticks right", prev_ticks_right);
        telemetry.addData("ticks left", prev_ticks_left);
        telemetry.addData("theta", theta);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("dx", dx);
        telemetry.addData("dy", dy);
        telemetry.addData("dtheta", dtheta);
        telemetry.addData("perpendicular", dx_perpendicular);
        telemetry.addData("center", dx_center);
        telemetry.addData("d_back", delta_ticks_back);
        telemetry.addData("d_left", delta_ticks_left);
        telemetry.addData("d_right", delta_ticks_right);
        telemetry.update();
        dashboard.sendTelemetryPacket(packet);

        prev_ticks_back = backRight.getCurrentPosition();
        prev_ticks_left = frontLeft.getCurrentPosition();
        prev_ticks_right = frontRight.getCurrentPosition();

    }
}