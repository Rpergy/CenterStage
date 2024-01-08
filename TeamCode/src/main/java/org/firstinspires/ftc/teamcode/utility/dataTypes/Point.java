package org.firstinspires.ftc.teamcode.utility.dataTypes;

public class Point {
    public double x, y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point(Point newPoint) {
        this.x = newPoint.x;
        this.y = newPoint.y;
    }

    public Point(Pose pose) {
        this.x = pose.x;
        this.y = pose.y;
    }

    public boolean equals(Point p) {
        return p.x == x && p.y == y;
    }

    public boolean withinRange(Point p, double range) {
        return (p.x >= x - range && p.x <= x + range) && (p.y >= y - range && p.y <= y + range);
    }
}
