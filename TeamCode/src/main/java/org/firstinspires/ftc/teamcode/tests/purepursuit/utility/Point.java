package org.firstinspires.ftc.teamcode.tests.purepursuit.utility;

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
}
