package org.firstinspires.ftc.teamcode.utility.dataTypes;

public class Point {
    public double x, y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point(Point replace) {
        this.x = replace.x;
        this.y = replace.y;
    }

    public boolean equals(Point p) {
        return p.x == x && p.y == y;
    }

    public boolean inRange(Point p, double range) {
        return (p.x >= x - range && p.x <= x + range) && (p.y >= y - range && p.y <= y + range);
    }
}
