package org.firstinspires.ftc.teamcode.utility.dataTypes;

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

    public Pose(Pose newPose) {
        this.x = newPose.x;
        this.y = newPose.y;
        this.heading = newPose.heading;
    }

    public Point toPoint() {
        return new Point(x, y);
    }

    public boolean inRange(Point p, double range) {
        return (p.x >= x - range && p.x <= x + range) && (p.y >= y - range && p.y <= y + range);
    }
}
