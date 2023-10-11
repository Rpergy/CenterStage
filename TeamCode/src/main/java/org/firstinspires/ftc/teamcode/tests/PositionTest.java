package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(group = "Tests", name = "Position Test")
public class PositionTest extends OpMode {
    BHI260IMU imu;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    double wheel_circ, ticksPerRev, track_width, k, forward_offset;
    double fwd, str;

    double x, y, theta;

    double prev_ticks_left = 0, prev_ticks_right = 0, prev_ticks_back = 0;

    double dx, dy, dtheta;

    double dx_center, dx_perpendicular;

    @Override
    public void init() {
        wheel_circ = 157.08; // mm
        track_width = 285; // mm
        forward_offset = 140; // mm
        k = 165.1; // mm
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

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {

        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        double[] xs = {x/100 -5, x/100 -5, x/100+5, x/100+5};
        double[] ys = {y/100 -5, y/100 +5, y/100+5, y/100-5};
        packet.fieldOverlay().fillPolygon(xs, ys).setFill("blue");

        double move = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;

        frontLeft.setPower(move - turn + strafe);
        frontRight.setPower(move + turn - strafe);
        backLeft.setPower(move - turn - strafe);
        backRight.setPower(move + turn + strafe);

        double delta_ticks_left = frontLeft.getCurrentPosition() - prev_ticks_left;
        double delta_ticks_right = frontRight.getCurrentPosition() - prev_ticks_right;
        double delta_ticks_back = backRight.getCurrentPosition() - prev_ticks_back;

//        double dist_left = ticks_left / ticksPerRev * wheel_circ;
//        double dist_right = ticks_right / ticksPerRev * wheel_circ;
//        double dist_back = ticks_back / ticksPerRev * wheel_circ;

        dtheta = (delta_ticks_left - delta_ticks_right) / track_width;
        dx_center = (delta_ticks_left + delta_ticks_right) / 2;
        dx_perpendicular = delta_ticks_back - (forward_offset * dtheta);

        dx = dx_center * Math.cos(theta) - dx_perpendicular * Math.sin(theta);
        dy = dx_center * Math.sin(theta) + dx_perpendicular * Math.cos(theta);

//        double dx += dx_center * Math.cos(theta) - dx_perpendicular * Math.sin(theta);
//        dougle dy += dx_center * Math.sin(theta) + dx_perpendicular * Math.cos(theta);
//        double dtheta += (delta_ticks_left - delta_ticks_right) / track_width;
        x += dx;
        y += dy;
        theta += dtheta;


//        theta = (dist_right-dist_left)/(2*(track_width/2));
//        fwd = ((dist_left + dist_right) / 2)/25.4;
//        str = (dist_back - k * theta)/25.4;
//
//        double r = Math.sqrt(Math.pow(fwd, 2) + Math.pow(str, 2));
//        double theta0 = Math.atan2(fwd, str);
//        x = r * Math.cos(theta0 - theta);
//        y = r * Math.sin(theta0 - theta);

        telemetry.addData("ticks back", prev_ticks_back);
        telemetry.addData("ticks right", prev_ticks_right);
        telemetry.addData("ticks left", prev_ticks_left);
//        telemetry.addData("fwd", fwd);
//        telemetry.addData("str", str);
        telemetry.addData("theta", theta/ ticksPerRev * wheel_circ);
//        telemetry.addData("theta0", theta0);
        telemetry.addData("x", x/ ticksPerRev * wheel_circ);
        telemetry.addData("y", y/ ticksPerRev * wheel_circ);
        telemetry.addData("dx", dx);
        telemetry.addData("dy", dy);
        telemetry.addData("dtheta", dtheta);
        telemetry.addData("perpendicular", dx_perpendicular);
        telemetry.addData("center", dx_center);
        telemetry.update();
        dashboard.sendTelemetryPacket(packet);

        prev_ticks_back = backRight.getCurrentPosition();
        prev_ticks_left = frontLeft.getCurrentPosition();
        prev_ticks_right = frontRight.getCurrentPosition();

    }
}