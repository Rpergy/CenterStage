package org.firstinspires.ftc.teamcode.utility.autonomous;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.lang.reflect.Field;
import java.util.Dictionary;
import java.util.Hashtable;

@Config
public class FieldConstants {
    Point mid = new Point(0, 0);

    public static class Blue {
        public static class Canvas {
            public static Pose center = new Pose(50, 34.5, Math.toRadians(180));
            public static Pose left = new Pose(50, 42, Math.toRadians(180));
            public static Pose right = new Pose(50, 26.5, Math.toRadians(180));
        }
    }
}
