package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.MathFunctions;
import org.firstinspires.ftc.teamcode.utility.RobotMovement;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;
import java.util.ArrayList;

@TeleOp(name="path test", group="tests")
public class PathTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotMovement robot = new RobotMovement(hardwareMap, new Pose(10, 66, Math.toRadians(-90)));

        ArrayList<Pose> startToCanvas = new ArrayList<>();
        startToCanvas.add(new Pose(10, 66, Math.toRadians(-90)));
        startToCanvas.add(new Pose(40, 66, Math.toRadians(0)));
        startToCanvas.add(new Pose(40, 40, Math.toRadians(0)));

        waitForStart();
        while (opModeIsActive()) {
            robot.displayPoses(startToCanvas, ActuationConstants.Autonomous.followDistance);
        }
    }
}
