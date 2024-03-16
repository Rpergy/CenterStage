package org.firstinspires.ftc.teamcode.tests.other;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.MathFunctions;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;

@TeleOp(name = "heading lock", group = "tests")
public class HeadingLock extends OpMode {
    FtcDashboard dashboard;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        AutoMovement.updatePosition();
        double heading = MathFunctions.AngleWrap(AutoMovement.robotPose.heading);

        if(gamepad1.circle) {
            double h1 = Math.abs(heading - Math.toRadians(0.0));
            double h2 = Math.abs(heading - Math.toRadians(90.0));
            double h3 = Math.abs(heading - Math.toRadians(-90.0));
            double h4 = Math.abs(heading - Math.toRadians(180.0));

                 if(h1 < h2 && h1 < h3 && h1 < h4) AutoMovement.turnTowards(Math.toRadians(0.0), 1.0);
            else if(h2 < h1 && h2 < h3 && h2 < h4) AutoMovement.turnTowards(Math.toRadians(90.0), 1.0);
            else if(h3 < h2 && h3 < h1 && h3 < h4) AutoMovement.turnTowards(Math.toRadians(-90.0), 1.0);
            else if(h4 < h2 && h4 < h3 && h4 < h1) AutoMovement.turnTowards(Math.toRadians(180.0), 1.0);
        }
        else {
            Actuation.drive(-gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
        }

        telemetry.addData("heading", Math.toDegrees(heading));
    }
}
