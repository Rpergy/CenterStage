package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@Config
public class FieldConstants {
    public static Pose mid = new Pose(0, 0, 0);
    public static Pose blueSpawnLeft = new Pose(-50, 25, 0);
    public static Pose blueSpawnRight = new Pose(-50, -25, 0);
    public static Pose blueCanvas = new Pose(-40, 55, Math.toRadians(90));
    public static Pose blueWing = new Pose(50, -50, Math.toRadians(-90));

    public static Pose redSpawnLeft;
    public static Pose redSpawnRight;
    public static Pose redCanvas;
    public static Pose redWing;
}
