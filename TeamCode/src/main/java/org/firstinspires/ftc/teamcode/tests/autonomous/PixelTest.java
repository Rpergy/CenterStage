package org.firstinspires.ftc.teamcode.tests.autonomous;

import static org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;

import java.util.Arrays;

@Autonomous(group = "tests", name = "PixelTest")
public class PixelTest extends OpMode {
    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
        Actuation.setIntakeArm(ActuationConstants.Intake.stackPos[0]);
    }
    @Override
    public void loop() {
        Actuation.setIntake(-1);
        Actuation.toggleDeposit(1);
        telemetry.addData("top", Arrays.toString(Actuation.getColorTop()));
        telemetry.addData("bottom", Arrays.toString(Actuation.getColorBottom()));
        telemetry.update();
    }

}
