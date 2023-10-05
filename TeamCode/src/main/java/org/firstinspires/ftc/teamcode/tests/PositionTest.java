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

@TeleOp(name = "Position Test")
public class PositionTest extends OpMode {
    BHI260IMU imu;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    double wheel_circ, ticksPerRev, track_width, k;
    double fwd, str, theta;

    double x, y;

    @Override
    public void init() {
        wheel_circ = 157.08; // mm
        track_width = 292.1; // mm
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
        packet.put("x", 3.7);
        double[] xs = {x-5, x-5, x+5, x+5};
        double[] ys = {y-5, y+5, y+5, y-5};
        packet.fieldOverlay().fillPolygon(xs, ys).setFill("blue");
//        packet.fieldOverlay()
//                .setFill("blue")
//                .fillRect(-20, -20, 40, 40);

        double move = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;

        frontLeft.setPower(move - turn + strafe);
        frontRight.setPower(move + turn - strafe);
        backLeft.setPower(move - turn - strafe);
        backRight.setPower(move + turn + strafe);

        double ticks_left = frontLeft.getCurrentPosition();
        double ticks_right = frontRight.getCurrentPosition();
        double ticks_back = backRight.getCurrentPosition();

        double dist_left = ticks_left / ticksPerRev * wheel_circ;
        double dist_right = ticks_right / ticksPerRev * wheel_circ;
        double dist_back = ticks_back / ticksPerRev * wheel_circ;

        theta = (dist_right-dist_left)/(2*(track_width/2));
        fwd = ((dist_left + dist_right) / 2)/25.4;
        str = (dist_back - k * theta)/25.4;

        double r = Math.sqrt(Math.pow(fwd, 2) + Math.pow(str, 2));
        double theta0 = Math.atan2(fwd, str);
        x = r * Math.cos(theta0 - theta);
        y = r * Math.sin(theta0 - theta);

        telemetry.addData("ticks left", ticks_left);
        telemetry.addData("ticks right", ticks_right);
        telemetry.addData("ticks back", ticks_back);
        telemetry.addData("fwd", fwd);
        telemetry.addData("str", str);
        telemetry.addData("theta", theta);
        telemetry.addData("theta0", theta0);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
    }
}

