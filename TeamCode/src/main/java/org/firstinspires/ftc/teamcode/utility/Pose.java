package org.firstinspires.ftc.teamcode.utility;

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
}
