package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.List;

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

    double scale;

    double start_time, end_time;
    public static double lateral_multiplier, center_multiplier, perpendicular_multiplier;

    List<LynxModule> allHubs;

    @Override
    public void init() {

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        wheel_circ = 6.184; // inches
        track_width = 11.024; // in distance between drive wheels
        forward_offset = -5.906; // in distance from center of robot to perp wheel
        ticksPerRev = 8192;

        lateral_multiplier = 1.033174886; //1.010112392;
        center_multiplier = 1; //2.05759425438;
        perpendicular_multiplier = 1;//1.2;

        track_width *= lateral_multiplier;

        scale = wheel_circ / ticksPerRev;

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
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        double ticks_left = frontLeft.getCurrentPosition();
        double ticks_right = frontRight.getCurrentPosition();
        double ticks_back = backRight.getCurrentPosition();

        start_time = System.nanoTime();

        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        double[] xs = {(side_length * Math.cos(theta) - side_length * Math.sin(theta)) + x,
                (-side_length * Math.cos(theta) - side_length * Math.sin(theta)) + x,
                (-side_length * Math.cos(theta) + side_length * Math.sin(theta)) + x,
                (side_length * Math.cos(theta) + side_length * Math.sin(theta)) + x};

        double[] ys = {(side_length * Math.sin(theta) + side_length * Math.cos(theta)) + y,
                (-side_length * Math.sin(theta) + side_length * Math.cos(theta)) + y,
                (-side_length * Math.sin(theta) - side_length * Math.cos(theta)) + y,
                (side_length * Math.sin(theta) - side_length * Math.cos(theta)) + y};

        packet.fieldOverlay().fillPolygon(xs, ys).setFill("blue");

        double move = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;

        frontLeft.setPower(move - turn + strafe);
        frontRight.setPower(move + turn - strafe);
        backLeft.setPower(move - turn - strafe);
        backRight.setPower(move + turn + strafe);

        double delta_ticks_left = (ticks_left - prev_ticks_left);
        double delta_ticks_right = (ticks_right - prev_ticks_right);
        double delta_ticks_back = (ticks_back - prev_ticks_back);

        dtheta = ((delta_ticks_left - delta_ticks_right) / track_width) * scale;
        dx_center = ((delta_ticks_left + delta_ticks_right) / 2) * scale * center_multiplier;
        dx_perpendicular = -1 * (delta_ticks_back - (forward_offset * ((delta_ticks_left - delta_ticks_right) / track_width))) * scale * perpendicular_multiplier;

        dx = dx_center * Math.cos(theta) - dx_perpendicular * Math.sin(theta);
        dy = dx_center * Math.sin(theta) + dx_perpendicular * Math.cos(theta);

        x += dx;
        y += dy;
        theta += -1 * dtheta;

        end_time = System.nanoTime();


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
        telemetry.addData("fps", 1000/((end_time - start_time) * 0.000001));
        telemetry.update();
        dashboard.sendTelemetryPacket(packet);

        prev_ticks_back = ticks_back;
        prev_ticks_left = ticks_left;
        prev_ticks_right = ticks_right;

    }
}