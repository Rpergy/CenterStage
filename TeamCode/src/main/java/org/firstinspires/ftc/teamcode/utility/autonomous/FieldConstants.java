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
            public static Pose transition = new Pose(11.5, 38, Math.toRadians(-90));

            public static Pose leftSpike = new Pose(23, 38, Math.toRadians(-90));
            public static Pose centerSpike = new Pose(11.5, 32, Math.toRadians(-90));
            public static Pose rightSpike = new Pose(3, 38, Math.toRadians(-90));
        }

        public static class Right {
            public static Pose start = new Pose(-38.5, 63, Math.toRadians(-90));
            public static Pose transition = new Pose(-38.5, 46, Math.toRadians(-90));

            public static Pose leftSpike = new Pose(-33, 34, Math.toRadians(-45));
            public static Pose centerSpike = new Pose(-38.5, 34, Math.toRadians(-90));
            public static Pose rightSpike = new Pose(-48.5, 38, Math.toRadians(-90));
        }

        public static class Canvas {
            public static Pose center = new Pose(45, 34.5, Math.toRadians(0));
            public static Pose left = new Pose(45, 42, Math.toRadians(0));
            public static Pose right = new Pose(45, 26.5, Math.toRadians(0));
        }

        public static class Stacks {
            public static Pose center = new Pose(0, 0, Math.toRadians(0));
            public static Pose left = new Pose(0, 0, Math.toRadians(0));
            public static Pose right = new Pose(-58, 38, Math.toRadians(0));
        }

        public static class Park {
            public static Pose left = new Pose(53, 61, Math.toRadians(0));
            public static Pose right = new Pose(0, 0, Math.toRadians(0));
        }
    }

    public static class Red {
        public static class Left {
            public static Pose start = new Pose(-36.5, -63, Math.toRadians(90));
            public static Pose transition = new Pose(-36.5, -43, Math.toRadians(90));

            public static Pose leftSpike = new Pose(-46.5, -36, Math.toRadians(-45));
            public static Pose centerSpike = new Pose(-36.5, -32, Math.toRadians(90));
            public static Pose rightSpike = new Pose(-26, -36, Math.toRadians(45));
        }

        public static class Right {
            public static Pose start = new Pose(11.5, -63, Math.toRadians(90));
            public static Pose transition = new Pose(11.5, -43, Math.toRadians(90));

            public static Pose leftSpike = new Pose(3.5, -36, Math.toRadians(135));
            public static Pose centerSpike = new Pose(11.5, -32, Math.toRadians(90));
            public static Pose rightSpike = new Pose(21.5, -38, Math.toRadians(90));
        }

        public static class Canvas {
            public static Pose center = new Pose(50, 34.5, Math.toRadians(0));
            public static Pose left = new Pose(50, 42, Math.toRadians(0));
            public static Pose right = new Pose(50, 26.5, Math.toRadians(0));
        }

        public static class Stacks {
            public static Pose center = new Pose(0, 0, Math.toRadians(0));
            public static Pose left = new Pose(-59, -36, Math.toRadians(0));
            public static Pose right = new Pose(0, 0, Math.toRadians(0));
        }

        public static class Park {
            public static Pose left = new Pose(60, -60, Math.toRadians(0));
            public static Pose right = new Pose(0, 0, Math.toRadians(0));
        }
    }
}