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
            public static Pose centerSpike = new Pose(11.5, 35, Math.toRadians(-90));
            public static Pose rightSpike = new Pose(8, 38, Math.toRadians(-135));
        }

        public static class Right {
            public static Pose start = new Pose(-38.5, 63, Math.toRadians(-90));
            public static Pose transition = new Pose(-38.5, 46, Math.toRadians(-90));

            public static Pose leftSpike = new Pose(-35, 38, Math.toRadians(-45));
            public static Pose centerSpike = new Pose(-38.5, 36, Math.toRadians(-90));
            public static Pose rightSpike = new Pose(-48.5, 38, Math.toRadians(-90));
        }

        public static class Canvas {
            public static Pose center = new Pose(43, 34.5, Math.toRadians(0));
            public static Pose left = new Pose(43, 42, Math.toRadians(0));
            public static Pose right = new Pose(43, 26.5, Math.toRadians(0));
        }

        public static class Stacks {
            public static Pose center = new Pose(0, 0, Math.toRadians(0));
            public static Pose left = new Pose(-57, 10, Math.toRadians(0));
            public static Pose right = new Pose(-57, 38, Math.toRadians(0));
        }

        public static class Park {
            public static Pose left = new Pose(53, 61, Math.toRadians(0));
            public static Pose right = new Pose(53, 11, Math.toRadians(0));
        }
    }

    public static class Red {
        public static class Left {
            public static Pose start = new Pose(-36.5, -63, Math.toRadians(90));
            public static Pose transition = new Pose(-36.5, -43, Math.toRadians(90));

            public static Pose leftSpike = new Pose(-41, -36, Math.toRadians(135));
            public static Pose centerSpike = new Pose(-36.5, -33, Math.toRadians(90));
            public static Pose rightSpike = new Pose(-34, -36, Math.toRadians(45));
        }

        public static class Right {
            public static Pose start = new Pose(11.5, -63, Math.toRadians(90));
            public static Pose transition = new Pose(11.5, -43, Math.toRadians(90));

            public static Pose leftSpike = new Pose(3.5, -36, Math.toRadians(135));
            public static Pose centerSpike = new Pose(11.5, -32, Math.toRadians(90));
            public static Pose rightSpike = new Pose(21.5, -38, Math.toRadians(90));
        }

        public static class Canvas {
            public static Pose center = new Pose(50, -36, Math.toRadians(0));
            public static Pose left = new Pose(50, -30, Math.toRadians(0));
            public static Pose right = new Pose(50, -42, Math.toRadians(0));
        }

        public static class Stacks {
            public static Pose center = new Pose(0, 0, Math.toRadians(0));
            public static Pose left = new Pose(-55, -36, Math.toRadians(0));
            public static Pose right = new Pose(0, 0, Math.toRadians(0));
        }

        public static class Park {
            public static Pose left = new Pose(60, -60, Math.toRadians(0));
            public static Pose right = new Pose(0, 0, Math.toRadians(0));
        }
    }
}