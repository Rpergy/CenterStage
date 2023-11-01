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
        ArrayList<Point> points = new ArrayList<>();

        double m = (linePoint2.y - linePoint1.y)/(linePoint2.x - linePoint1.x);
        double r = radius;
        double b = linePoint1.y - m * linePoint1.x;
        double w = -circleCenter.x;
        double c = -circleCenter.y;

        double discriminant = -Math.pow(b, 2) + 2 * m * b * w - 2 * b * c + Math.pow(m, 2) * Math.pow(r, 2) + 2 * m * c * w + Math.pow(r, 2) - Math.pow(m, 2) * Math.pow(w, 2) - Math.pow(c, 2);

        if (discriminant >= 0){
            double xRoot1 = -(m*b+m*c+w + Math.sqrt(discriminant))/(Math.pow(m, 2) + 1);
            double xRoot2 = -(m*b+m*c+w - Math.sqrt(discriminant))/(Math.pow(m, 2) + 1);

            double y1 = m * xRoot1 + b;
            double y2 = m * xRoot2 + b;

            points.add(new Point(xRoot1, y1));
            if (discriminant != 0){
                points.add(new Point(xRoot2, y2));
            }
        }
        double minX = Math.min(linePoint1.x, linePoint2.x);
        double maxX = Math.max(linePoint1.x, linePoint2.x);

        ArrayList<Point> intersections = new ArrayList<>();
        for(int i = 0; i < points.size(); i++) {
            if (points.get(i).x <= maxX && points.get(i).x >= minX) {
                intersections.add(points.get(i));
            }
        }

        return intersections;
    }
}
