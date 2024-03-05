package org.firstinspires.ftc.teamcode.utility.autonomous;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.lang.reflect.Field;
import java.util.Dictionary;
import java.util.Hashtable;

@Config
public class FieldConstants {
    Point middle = new Point(0, 0);

    public static class Blue {
        public static class Left {
            public static Pose start = new Pose(11.5, 63, Math.toRadians(-90));
            public static Pose transition = new Pose(11.5, 52, Math.toRadians(-90));

            public static Pose leftSpike = new Pose(21, 41, Math.toRadians(-90));
            public static Pose centerSpike = new Pose(11.5, 36, Math.toRadians(-90));
            public static Pose rightSpike = new Pose(9, 36, Math.toRadians(-180));
        }

        public static class Right {
            public static Pose start = new Pose(-38.5, 63, Math.toRadians(-90));
            public static Pose transition = new Pose(-38.5, 46, Math.toRadians(-90));

            public static Pose leftSpike = new Pose(-37, 37, Math.toRadians(0));
            public static Pose centerSpike = new Pose(-38.5, 36, Math.toRadians(-90));
            public static Pose rightSpike = new Pose(-48.5, 36.5, Math.toRadians(-90));
        }

        public static class Canvas {
            public static Pose center = new Pose(42, 38.5, Math.toRadians(0));
            public static Pose left = new Pose(42, 44.5, Math.toRadians(0));
            public static Pose right = new Pose(42, 32.5, Math.toRadians(0));
        }

        public static class Stacks {
            public static Pose center = new Pose(0, 0, Math.toRadians(0));
            public static Pose left = new Pose(-57, 10, Math.toRadians(0));
            public static Pose right = new Pose(-60, 37, Math.toRadians(0));
        }

        public static class Park {
            public static Pose left = new Pose(56, 60.5, Math.toRadians(0));
            public static Pose right = new Pose(56, 13, Math.toRadians(0));
        }
    }

    public static class Red {
        public static class Left {
            public static Pose start = new Pose(-36.5, -63, Math.toRadians(90));
            public static Pose transition = new Pose(-36.5, -47, Math.toRadians(90));

            public static Pose leftSpike = new Pose(-48, -42, Math.toRadians(90));
            public static Pose centerSpike = new Pose(-36.5, -37, Math.toRadians(90));
            public static Pose rightSpike = new Pose(-35, -36, Math.toRadians(0));
        }

        public static class Right {
            public static Pose start = new Pose(11.5, -63, Math.toRadians(90));
            public static Pose transition = new Pose(11.5, -50, Math.toRadians(90));

            public static Pose leftSpike = new Pose(10, -36, Math.toRadians(180));
            public static Pose centerSpike = new Pose(11.5, -35.5, Math.toRadians(90));
            public static Pose rightSpike = new Pose(25, -42, Math.toRadians(90));
        }

        public static class Canvas {
            public static Pose center = new Pose(39, -39, Math.toRadians(0));
            public static Pose left = new Pose(37, -31, Math.toRadians(0));
            public static Pose right = new Pose(37, -42.5, Math.toRadians(0));
        }

        public static class Stacks {
            public static Pose center = new Pose(0, 0, Math.toRadians(0));
            public static Pose left = new Pose(-55, -36, Math.toRadians(0));
            public static Pose right = new Pose(0, 0, Math.toRadians(0));
        }

        public static class Park {
            public static Pose right = new Pose(50, -60, Math.toRadians(0));
            public static Pose left = new Pose(50, -12, Math.toRadians(0));
        }
    }
}