package org.firstinspires.ftc.teamcode.purepursuit.utility;

import java.util.ArrayList;

public class MathFunctions {
    /**
     * Makes sure an angle is within the range -3.14 to 3.14 radians
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

    /**
     * Returns a list of intersections between a line and a sphere
     * @param circleCenter Center of the circle
     * @param radius Radius of the circle
     * @param linePoint1 Point one of the line
     * @param linePoint2 Point two of the line
     * @return List of intersection points
     */
    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2) {
        // If ys are really close we almost have a vertical line
        if(Math.abs(linePoint1.y - linePoint2.y) < 0.003) linePoint1.y = linePoint2.y + 0.03;

        // If xs are really close we almost have horizontal
        if (Math.abs(linePoint1.x - linePoint2.x) < 0.003) linePoint2.x = linePoint2.x + 0.003;

        double m1 = (linePoint2.y - linePoint1.y)/(linePoint2.x - linePoint1.x);

        // Applies offset relative to the circles center to make calculations easier
        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        double a = 1.0 + Math.pow(m1, 2);
        double b = (2.0 * m1 * y1) - (2.0 * Math.pow(m1, 2) * x1);
        double c = ((Math.pow(m1, 2) * Math.pow(x1, 2))) - (2.0 * y1 * m1 * x1) + Math.pow(y1, 2) - Math.pow(radius, 2);

        ArrayList<Point> allPoints = new ArrayList<>();

        try {
            double xRoot1 = (-b + Math.sqrt(Math.pow(b, 2) - 4 * a * c))/(2.0 * a);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;

            // Re-apply circle offset
            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            // Calculate line bounding box
            double minX = Math.min(linePoint1.x, linePoint2.x);
            double maxX = Math.max(linePoint1.x, linePoint2.x);

            if (xRoot1 > minX && xRoot1 < maxX) {
                allPoints.add(new Point(xRoot1, yRoot1));
            }

            double xRoot2 = (-b - Math.sqrt(Math.pow(b, 2) - 4.0 * a * c))/(2.0 * a);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;

            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;

            if (xRoot2 > minX && xRoot2 < maxX) {
                allPoints.add(new Point(xRoot2, yRoot2));
            }
        }
        catch (Exception e){}

        return allPoints;
    }
}
