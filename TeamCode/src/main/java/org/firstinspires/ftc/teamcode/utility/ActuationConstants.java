package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

public class ActuationConstants {
    @Config
    public static class Autonomous {
        public static Pose robotStart = new Pose(-50, 25, 0);
        public static double moveSpeed = 0.5;
        public static double turnSpeed = 0.3;
        public static double followDistance = 15;
        //omkar is gay
        public static double minTurnSpeed = 0.05;
        public static double moveAccelMult = 0.2;
        public static double strafeAccelMult = 0.9;
    }

    @Config
    public static class Drivetrain {
        public static final double ticksPerRev = 2000;

        public static double center_multiplier = 0.3863728657; // responsible for move (test robot: 1.08
        public static double lateral_multiplier = 2.514924912; // responsible for turn (test robot: 1.033174886)
        public static double perpendicular_multiplier = 0.3902688729; // responsible for strafe (test robot: 1.06)

        public static double wheel_circ = 15.07; // cm
        public static double track_width = 12.25 * lateral_multiplier; // inches distance between drive wheels (test robot: 11.024)
        public static double forward_offset = 2.5; // inches distance from center of robot to perp wheel (test robot: -5.906)

        public static double scale = wheel_circ / ticksPerRev;
    }

    @Config
    public static class Claw {
        public static double wristIntake = 0.3;
        public static double wristDeposit = 0.8;
        public static double wristAutoInit = 1.0;
        public static double open = 1.0;
        public static double closed = 0.9;
    }

    @Config
    public static class Extension {
        public static int maxExtension = 2270;
        public static int extensionStart = 0;

        public static double tiltIntake = 0.15;

        public static int[] extensionPresets = {
                500, // level 1
                900, // level 2
                1200, // level 3
                0 // intake
        };

        public static double[] tiltPresets = { // 0.15 (intake) - 0.55 (closest deposit)
                0.55, // level 1
                0.50, // level 2
                0.45, // level 3
                0.15 // level 4
        };
        //omkar is gay amog us
        //I have hrard throug the grapevine that shreyas is the opps!!!!!!!!!
        //be sure to comment on your code!!!!!!!!!!!!!!
    }
}
