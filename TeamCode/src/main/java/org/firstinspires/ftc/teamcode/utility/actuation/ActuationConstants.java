package org.firstinspires.ftc.teamcode.utility.actuation;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

public class ActuationConstants {
    @Config
    public static class Autonomous {
        public static Pose robotStart = new Pose(0, 0, 0);
        public static double moveSpeed = 0.3;
        public static double turnSpeed = 0.4;
        public static double followDistance = 10;
    }

    @Config
    public static class Drivetrain {
        public static final double ticksPerRev = 8192;

        public static double center_multiplier = 1.08;
        public static double lateral_multiplier = 1.033174886;
        public static double perpendicular_multiplier = 1.06;

        public static double wheel_circ = 6.184; // inches
        public static double track_width = 11.024 * lateral_multiplier; // inches distance between drive wheels
        public static double forward_offset = -5.906; // inches distance from center of robot to perp wheel

        public static double scale = wheel_circ / ticksPerRev;
    }

    @Config
    public static class Claw {
        public static double wristDown = 0.0;
        public static double wristUp = 0.0;
        public static double open = 0.0;
        public static double closed = 0.0;
    }

    @Config
    public static class Extension {
        public static double[] targetPositions = {
                0.0, // down
                0.0, // midway
                0.0  // all out
        };
    }
}