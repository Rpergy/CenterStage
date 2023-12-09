package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.MathFunctions;
import org.firstinspires.ftc.teamcode.utility.RobotMovement;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;
import java.util.ArrayList;

@Autonomous(name="left blue")
public class LeftBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap);
        Actuation.setClaw(ActuationConstants.Claw.closed);
        Actuation.setWrist(ActuationConstants.Claw.wristDeposit);
        Actuation.setTilt(0.3);

        RobotMovement robot = new RobotMovement(hardwareMap, new Pose(-50, 30, 0));

        ArrayList<Pose> startToCanvas = new ArrayList<>();
        startToCanvas.add(new Pose(-50, 30, 0));
        startToCanvas.add(new Pose(-30, 40, Math.toRadians(90)));
        startToCanvas.add(new Pose(-30, 20, Math.toRadians(90)));

        waitForStart();
        while (MathFunctions.distance(startToCanvas.get(2), robot.robotPose) > 0.1) {
            robot.followPoseCurve(startToCanvas, ActuationConstants.Autonomous.followDistance, ActuationConstants.Autonomous.moveSpeed, ActuationConstants.Autonomous.turnSpeed);
            robot.updatePosition();
            robot.displayPoses(startToCanvas, ActuationConstants.Autonomous.followDistance);
        }
    }
}
