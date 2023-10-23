package org.firstinspires.ftc.teamcode.purepursuit.utility;

public class Pose {
    public double x, y, heading;

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose(Point point, double heading) {
        this.x = point.x;
        this.y = point.y;
        this.heading = heading;
    }

    public Point toPoint() {
        return new Point(x, y);
    }
}
