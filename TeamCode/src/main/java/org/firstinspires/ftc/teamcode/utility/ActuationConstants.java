package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;

public class ActuationConstants {
    @Config
    public static class Autonomous {
        public static double moveSpeed = 0.8;
        public static double turnSpeed = 0.85;
        public static double followDistance = 10;
        //omkar is gay
        public static double minTurnSpeed = 0.06;
        public static double turnAccelMult = 2.5;
        public static double moveAccelMult = 0.75;
        public static double strafeAccelMult = 0.75;
    }

    @Config
    public static class Drivetrain {
        public static final double ticksPerRev = 2000;

        public static double centerMultiplier = 0.3863728657; // responsible for move (test robot: 1.08)
        public static double lateral_multiplier = 2.34638397; // responsible for turn (test robot: 1.033174886)
        public static double perpendicularMultiplier = -0.39372002180; // responsible for strafe (test robot: 1.06)

        public static double wheel_circ = 15.07; // cm
        public static double track_width = 12.25 * lateral_multiplier; // inches distance between drive wheels (test robot: 11.024)
        public static double forward_offset = -15; // inches distance from center of robot to perp wheel (test robot: -5.906)

        public static double scale = wheel_circ / ticksPerRev;
    }

    @Config
    public static class Extension {
        public static int maxExtend = 2500;
        public static int period = 20;

        public static int hang = 700;

        public static double[] tiltPositions = {
                0.45, // intake
                0.52,
                0.70  // hang
        };

        public static int[] slidePositions = {
                0, // intake
                0,
                0,
                0 // highest
        };
    }

    @Config
    public static class Intake {
        public static double power = 0.0;
        public static double[] stackPos = {
                0.9,  // 1 pixel (ground)
                0.45, // 2 pixels
                0.300, // 3 pixels
                0.180, // 4 pixels
                0.075,  // 5 pixels
                0.0 // highest
        };

        public static double testStackPos = 0.0;
    }

    @Config
    public static class Deposit {
        public static double intakeTilt = 0.44;
        public static double depositTilt = 0.77;

        public static double open = 0.3;
        public static double closed = 0.0;
    }

    @Config
    public static class Plane {
        public static double releaseDown = 0.7;
        public static double releaseUp = 0.0;
        public static double tilt = 0.629;
    }
}
