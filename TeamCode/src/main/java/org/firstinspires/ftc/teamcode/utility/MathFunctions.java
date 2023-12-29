package org.firstinspires.ftc.teamcode.utility;

import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;

public class MathFunctions {
    /**
     * Makes sure an angle is within the range -PI to PI radians
     * @param angle radians
     * @return Wrapped angle
     */
    public static double AngleWrap(double angle) {
        while (angle < -Math.PI) {
            angle += Math.PI * 2;
        }
        while (angle > Math.PI) {
            angle -= Math.PI * 2;
        }

        return angle;
    }

    public static double lerp(double a, double b, double f) {
        return a * (1.0 - f) + (b * f);
    }


    /**
     * Returns a list of intersections between a line and a sphere
     * @param circleCenter Center of the circle
     * @param radius Radius of the circle
     * @param linePoint1 Point one of the line
     * @param linePoint2 Point two of the line
     * @return List of intersection points (0-2)
     */
    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2) {
        ArrayList<Point> intersections = new ArrayList<>();
        ArrayList<Point> points = new ArrayList<>();

        if (linePoint1.x == linePoint2.x) linePoint1.x += 0.01;

        double m = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);
        double r = radius;
        double b = linePoint1.y - m * linePoint1.x;
        double w = -circleCenter.x;
        double c = -circleCenter.y;

        double discriminant = -Math.pow(b, 2) + 2 * m * b * w - 2 * b * c + Math.pow(m, 2) * Math.pow(r, 2) + 2 * m * c * w + Math.pow(r, 2) - Math.pow(m, 2) * Math.pow(w, 2) - Math.pow(c, 2);

        if (discriminant >= 0) {
            double xRoot1 = -(m * b + m * c + w + Math.sqrt(discriminant)) / (Math.pow(m, 2) + 1);
            double xRoot2 = -(m * b + m * c + w - Math.sqrt(discriminant)) / (Math.pow(m, 2) + 1);

            double y1 = m * xRoot1 + b;
            double y2 = m * xRoot2 + b;

            points.add(new Point(xRoot1, y1));
            if (discriminant != 0) {
                points.add(new Point(xRoot2, y2));
            }
        }
        double minX = Math.min(linePoint1.x, linePoint2.x);
        double maxX = Math.max(linePoint1.x, linePoint2.x);

        for (int i = 0; i < points.size(); i++) {
            if (points.get(i).x <= maxX && points.get(i).x >= minX) {
                intersections.add(points.get(i));
            }
        }

        return intersections;
    }

    public static ArrayList<Pose> lineCircleIntersection(Point circleCenter, double radius, Pose linePoint1, Pose linePoint2) {
        ArrayList<Pose> intersections = new ArrayList<>();
        ArrayList<Point> points = new ArrayList<>();

        if (linePoint1.x == linePoint2.x) linePoint1.x += 0.01;

        double m = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);
        double r = radius;
        double b = linePoint1.y - m * linePoint1.x;
        double w = -circleCenter.x;
        double c = -circleCenter.y;

        double discriminant = -Math.pow(b, 2) + 2 * m * b * w - 2 * b * c + Math.pow(m, 2) * Math.pow(r, 2) + 2 * m * c * w + Math.pow(r, 2) - Math.pow(m, 2) * Math.pow(w, 2) - Math.pow(c, 2);

        if (discriminant >= 0) {
            double xRoot1 = -(m * b + m * c + w + Math.sqrt(discriminant)) / (Math.pow(m, 2) + 1);
            double xRoot2 = -(m * b + m * c + w - Math.sqrt(discriminant)) / (Math.pow(m, 2) + 1);

            double y1 = m * xRoot1 + b;
            double y2 = m * xRoot2 + b;

            points.add(new Point(xRoot1, y1));
            if (discriminant != 0) {
                points.add(new Point(xRoot2, y2));
            }
        }
        double minX = Math.min(linePoint1.x, linePoint2.x);
        double maxX = Math.max(linePoint1.x, linePoint2.x);

        for (int i = 0; i < points.size(); i++) {
            if (points.get(i).x <= maxX && points.get(i).x >= minX) {
                double heading = linePoint2.heading;
                intersections.add(new Pose(points.get(i), heading));
            }
        }

        return intersections;
    }

    public static double distance(Point p1, Point p2) {
        return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));
    }

    public static double distance(Pose p1, Pose p2) {
        return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2) + Math.pow(p1.heading - p2.heading, 2));
    }

    public static double cosineDistance(Point p1, Point p2){
        return (p1.x * p2.x + p1.y * p2.y)/((Math.sqrt(Math.pow(p1.x, 2) + Math.pow(p1.y, 2)) * Math.sqrt(Math.pow(p2.x, 2) + Math.pow(p2.y, 2))));
    }
}
