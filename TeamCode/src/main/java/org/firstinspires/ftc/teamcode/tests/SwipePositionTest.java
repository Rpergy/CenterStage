package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class SwipePositionTest extends OpMode {

    double[] initialPos;
    double[] finalPos;

    Servo servo;
    int iterationsSinceLastTouch;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo1");
        iterationsSinceLastTouch = Integer.MAX_VALUE;
        initialPos[0] = 0;
        initialPos[1] = 0;
        finalPos[0] = 0;
        finalPos[1] = 0;
    }

    @Override
    public void loop() {
        if (iterationsSinceLastTouch > 10 && gamepad1.touchpad) {
            initialPos[0] = gamepad1.touchpad_finger_1_x;
            initialPos[1] = gamepad1.touchpad_finger_1_y;
            iterationsSinceLastTouch = 0;
        }

        if (initialPos[0] != 0) {
            if (gamepad1.touchpad) {
                finalPos[0] = gamepad1.touchpad_finger_1_x;
                finalPos[1] = gamepad1.touchpad_finger_1_y;
                iterationsSinceLastTouch = 0;
            } else {
                // assuming 210 degree rotation
                double maxPower = 90.0 / 210;

                finalPos[0] = finalPos[0] - initialPos[0];
                finalPos[1] = finalPos[1] - initialPos[1];
                initialPos[0] = 0;
                initialPos[1] = 0;

                double degrees = Math.toDegrees(Math.atan(finalPos[0]/finalPos[1]));
                double power = (degrees / 90) * maxPower;

                servo.setPosition(power);
            }
        }

        iterationsSinceLastTouch++;
    }
}
