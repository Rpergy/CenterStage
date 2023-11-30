package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.lang.reflect.Field;
import java.util.Dictionary;
import java.util.Hashtable;

@Config
public class FieldConstants {
    public static enum Waypoint {
        MID,
        CENTER_CANVAS,
        CENTER_WING,
        BLUE_SPAWN_LEFT,
        BLUE_SPAWN_RIGHT,
        BLUE_CANVAS,
        BLUE_WING,
        RED_SPAWN_LEFT,
        RED_SPAWN_RIGHT,
        RED_CANVAS,
        RED_WING
    }

    public static Pose[] waypointPositions = {
            new Pose(  0,   0, 0), // mid
            new Pose(  0,  50, 0), // center canvas
            new Pose(  0, -50, 0), // center wing
            new Pose(-50,  25, 0), // blue spawn left
            new Pose(-50, -25, 0), // blue spawn right
            new Pose(-40,  55, Math.toRadians(90)), // blue canvas
            new Pose( 50, -50, Math.toRadians(-90)), // blue wing
            new Pose( 50, -25, Math.toRadians(180)), // red spawn left
            new Pose( 50,  25, Math.toRadians(180)), // red spawn right
            new Pose( 40,  55, Math.toRadians(90)), // red canvas
            new Pose(-50, -50, Math.toRadians(180)) // red wing
    };

    public static int[][] waypointConnections = {
            {0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0}, // mid
            {1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0}, // center canvas
            {1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1}, // center wing
            {0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0}, // blue spawn left
            {0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1}, // blue spawn right
            {0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0}, // blue canvas
            {0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1}, // blue wing
            {0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0}, // red spawn left
            {0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0}, // red spawn right
            {0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0}, // red canvas
            {0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0} // red wing
    };
}
