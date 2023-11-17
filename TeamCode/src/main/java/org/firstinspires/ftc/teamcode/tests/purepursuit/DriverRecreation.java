package org.firstinspires.ftc.teamcode.tests.purepursuit;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.tests.purepursuit.utility.Point;
import org.firstinspires.ftc.teamcode.tests.purepursuit.utility.Pose;

import com.arcrobotics.ftclib.gamepad.*;

import java.util.ArrayList;

@TeleOp(name = "DriverRecreation")
public class DriverRecreation extends OpMode {
    RobotMovement robot;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    double start_time, end_time, totalTime;

    double pollTime;

    ArrayList<Point> pathPoints;

    boolean pollPoints, pollPressedToggle, driving, drivePressedToggle;

    double moveSpeed, turnSpeed, followDistance;

    GamepadEx gamepadEx;

    @Override
    public void init() {
        robot = new RobotMovement(hardwareMap, new Pose(0, 0, 0));

        pollTime = 0.1;

        pollPoints = false;
        driving = true;

        pathPoints = new ArrayList<>();

        gamepadEx = new GamepadEx(gamepad1);

        moveSpeed = 0.3;
        turnSpeed = 0.7;
        followDistance = 10;

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
    }

    public void loop() {
        start_time = System.currentTimeMillis() / 1000.0;

        robot.updatePosition(telemetry);

        if (gamepad1.square && !drivePressedToggle) {
            driving = !driving;
            drivePressedToggle = true;
        }
        else if (!gamepad1.square){
            drivePressedToggle = false;
        }

        if (driving) {
            double move = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;

            frontLeft.setPower(move - turn - strafe);
            frontRight.setPower(move + turn + strafe);
            backLeft.setPower(move - turn + strafe);
            backRight.setPower(move + turn - strafe);

            if (gamepad1.triangle && !pollPressedToggle) {
                pollPoints = !pollPoints;
                pollPressedToggle = true;
            } else if (!gamepad1.triangle) {
                pollPressedToggle = false;
            }

            end_time = System.currentTimeMillis() / 1000.0;

            totalTime += end_time - start_time;

            if (totalTime >= pollTime && pollPoints) {
                pathPoints.add(robot.robotPose.toPoint());
                totalTime = 0;
            }
        }
        else {
            robot.followCurve(pathPoints, followDistance, moveSpeed, turnSpeed);
        }

        telemetry.addData("Polling", pollPoints);
        telemetry.addData("Driving", driving);
        telemetry.addData("Path Points", pathPoints.size());
        telemetry.update();
    }
}
