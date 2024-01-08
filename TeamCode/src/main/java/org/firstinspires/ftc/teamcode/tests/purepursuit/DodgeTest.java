package org.firstinspires.ftc.teamcode.tests.purepursuit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.MathFunctions;
import org.firstinspires.ftc.teamcode.utility.autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.RobotMovement;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Dodge Test")
public class DodgeTest extends OpMode {
    DodgeMovement robot;

    @Override
    public void init() {
        robot = new DodgeMovement(hardwareMap, new Pose(11.5, 63, -90));
    }

    @Override
    public void loop() {
        robot.updatePosition();

        ArrayList<Pose> allPoses = new ArrayList<Pose>();
        allPoses.add(new Pose(11.5, 0, -90));

        robot.incrementPoseCurve(allPoses, ActuationConstants.Autonomous.followDistance, ActuationConstants.Autonomous.moveSpeed, ActuationConstants.Autonomous.turnSpeed);
        robot.displayPoses(allPoses, ActuationConstants.Autonomous.followDistance);
    }
}