package org.firstinspires.ftc.teamcode.utility;

public class CurvePoint {
    double x, y, heading, followDistance;

    public CurvePoint (double x, double y, double heading, double followDistance) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.followDistance = followDistance;
    }

    public CurvePoint (Point p, double heading, double followDistance) {
        this.x = p.x;
        this.y = p.y;
        this.heading = heading;
        this.followDistance = followDistance;
    }

    public CurvePoint (Pose p, double followDistance) {
        this.x = p.x;
        this.y = p.y;
        this.heading = p.heading;
        this.followDistance = followDistance;
    }

    public CurvePoint (double x, double y) {
        this.x = x;
        this.y=  y;
    }

    public CurvePoint (double x, double y, double followDistance) {
        this.x = x;
        this.y = y;
        this.followDistance = followDistance;
    }

    public CurvePoint (CurvePoint muthu) {
        x = muthu.x;
        y = muthu.y;
        heading = muthu.heading;
        followDistance = muthu.followDistance;
    }

    public Point toPoint() {
        return new Point(x, y);
    }

    public Pose toPose() {
        return new Pose(x, y, heading);
    }

    public void muthuParty() {
        while(true) {
            System.out.println("I LOVE MUTHUKALAYAPPAN");
        }
    }
}
