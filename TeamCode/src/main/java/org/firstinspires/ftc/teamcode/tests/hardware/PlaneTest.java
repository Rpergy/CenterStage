package org.firstinspires.ftc.teamcode.tests.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.ActuationConstants;

@TeleOp(name="Plane Test", group="tests")
public class PlaneTest extends OpMode {
    Servo tilt, release;

    @Override
    public void init() {
        tilt = hardwareMap.servo.get("planeTilt");
        release = hardwareMap.servo.get("planeRelease");

        tilt.setPosition(0.0);
        release.setPosition(ActuationConstants.Plane.releaseDown);
    }
// The only motion in the ocean is going to be Shreyas' flailing arms as I drown him in the sea
    @Override
    public void loop() {
        tilt.setPosition(ActuationConstants.Plane.tilt);

        if(gamepad1.ps) release.setPosition(ActuationConstants.Plane.releaseUp);
        else release.setPosition(ActuationConstants.Plane.releaseDown);

        telemetry.addData("tilt pos", tilt.getPosition());
        telemetry.addData("release pos", release.getPosition());
        telemetry.update();
    }
}
// Jason ate my dog