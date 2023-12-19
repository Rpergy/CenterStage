package org.firstinspires.ftc.teamcode.utility.tests.purepursuit;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;
import org.firstinspires.ftc.teamcode.utility.RobotMovement;

import java.util.ArrayList;

@TeleOp(name = "DriverRecreation")
@Disabled
public class DriverRecreation extends OpMode {
    RobotMovement robot;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    double start_time, end_time, totalTime;

    double pollTime;

    ArrayList<Pose> pathPoints;

    boolean pollPoints, pollPressedToggle, driving, drivePressedToggle;

    double moveSpeed, turnSpeed, followDistance;

    @Override
    public void init() {
        robot = new RobotMovement(hardwareMap, ActuationConstants.Autonomous.robotStart);

        pollTime = 0.1;

        pollPoints = false;
        driving = true;

        pathPoints = new ArrayList<>();

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

        pathPoints.add(new Pose(robot.robotPose));
    }

    public void loop() {
        start_time = System.currentTimeMillis() / 1000.0;

        robot.updatePosition();

        if (gamepad1.cross && !drivePressedToggle) {
            driving = !driving;
            drivePressedToggle = true;
        }
        else if (!gamepad1.cross){
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
                pathPoints.add(0, new Pose(robot.robotPose));
                totalTime = 0;
            }
        }
        else {
            robot.incrementPoseCurve(pathPoints, followDistance, moveSpeed, turnSpeed);
            robot.displayPoses(pathPoints, followDistance);
        }

        telemetry.addData("Polling", pollPoints);
        telemetry.addData("Driving", driving);
        telemetry.addData("Path Points", pathPoints.size());
        telemetry.update();
    }
}
