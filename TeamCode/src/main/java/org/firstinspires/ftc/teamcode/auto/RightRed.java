package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.RobotMovement;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;
import java.util.ArrayList;

@Autonomous(name="right red")
public class RightRed extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap);
        Actuation.setWrist(ActuationConstants.Claw.wristAutoInit);
        Actuation.setTilt(0.3);

        RobotMovement robot = new RobotMovement(hardwareMap, new Pose(10, -66, Math.toRadians(90)));

        ArrayList<Pose> startToCanvas = new ArrayList<>();
        startToCanvas.add(new Pose(10, -66, Math.toRadians(90)));
        startToCanvas.add(new Pose(50, -66, Math.toRadians(0)));

        waitForStart();
        while (opModeIsActive()) {
            robot.updatePosition();
            robot.incrementPoseCurve(startToCanvas, ActuationConstants.Autonomous.followDistance, ActuationConstants.Autonomous.moveSpeed, ActuationConstants.Autonomous.turnSpeed);
            robot.displayPoses(startToCanvas, ActuationConstants.Autonomous.followDistance);
            if (robot.robotPose.withinRange(startToCanvas.get(1), 15.0)) {
                Actuation.setWrist(ActuationConstants.Claw.wristIntake);
            }
            if (robot.robotPose.withinRange(startToCanvas.get(1), 3.0)) {
                Actuation.setClaw(ActuationConstants.Claw.open);
            }
        }
    }
}
