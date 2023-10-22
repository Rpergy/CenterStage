package org.firstinspires.ftc.teamcode.purepursuit;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class PurePursuit extends OpMode {
    RobotMovement robot;

    @Override
    public void init() {
        robot = new RobotMovement(hardwareMap, new Pose(0, 0, 0));
    }

    @Override
    public void loop() {
        robot.updatePosition();

        robot.goToPosition(new Pose(100, 100, 90), 0.5, 0.5);
    }
}
