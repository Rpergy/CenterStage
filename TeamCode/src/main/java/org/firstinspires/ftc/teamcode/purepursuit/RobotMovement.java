package org.firstinspires.ftc.teamcode.purepursuit;

import static org.firstinspires.ftc.teamcode.purepursuit.Functions.AngleWrap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class RobotMovement {
    DcMotor frontLeft, frontRight, backLeft, backRight;

    public static double wheel_circ, ticksPerRev, track_width, forward_offset;
    Pose worldPos;
    double prev_ticks_left = 0, prev_ticks_right = 0, prev_ticks_back = 0;
    double dx, dy, dtheta;
    double dx_center, dx_perpendicular;
    double side_length = 5;
    double scale;
    public static double lateral_offset;

    public RobotMovement(HardwareMap hardwareMap, double posX, double posY) {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        wheel_circ = 15.708; // cm
        track_width = 29.53/2; // cm distance between drive wheels
        forward_offset = -14.5; // cm distance from center of robot to perp wheel
        ticksPerRev = 8192;

        lateral_offset = 0.5;

        scale = wheel_circ / ticksPerRev;
    }

    public void updatePosition() {
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        double[] xs = {(side_length * Math.cos(worldPos.heading) - side_length * Math.sin(worldPos.heading)) + worldPos.x,
                (-side_length * Math.cos(worldPos.heading) - side_length * Math.sin(worldPos.heading)) + worldPos.x,
                (-side_length * Math.cos(worldPos.heading) + side_length * Math.sin(worldPos.heading)) + worldPos.x,
                (side_length * Math.cos(worldPos.heading) + side_length * Math.sin(worldPos.heading)) + worldPos.x};

        double[] ys = {(side_length * Math.sin(worldPos.heading) + side_length * Math.cos(worldPos.heading)) + worldPos.y,
                (-side_length * Math.sin(worldPos.heading) + side_length * Math.cos(worldPos.heading)) + worldPos.y,
                (-side_length * Math.sin(worldPos.heading) - side_length * Math.cos(worldPos.heading)) + worldPos.y,
                (side_length * Math.sin(worldPos.heading) - side_length * Math.cos(worldPos.heading)) + worldPos.y};

        packet.fieldOverlay().fillPolygon(xs, ys).setFill("blue");

        double delta_ticks_left = (frontLeft.getCurrentPosition() - prev_ticks_left);
        double delta_ticks_right = (frontRight.getCurrentPosition() - prev_ticks_right);
        double delta_ticks_back = (backRight.getCurrentPosition() - prev_ticks_back);

        dtheta = ((delta_ticks_left - delta_ticks_right) / track_width) * scale;
        dx_center = ((delta_ticks_left + delta_ticks_right) / 2) * scale;
        dx_perpendicular = (delta_ticks_back - (forward_offset * ((delta_ticks_left - delta_ticks_right) / track_width))) * scale * lateral_offset;

        dx = dx_center * Math.cos(worldPos.heading) - dx_perpendicular * Math.sin(worldPos.heading);
        dy = dx_center * Math.sin(worldPos.heading) + dx_perpendicular * Math.cos(worldPos.heading);

        worldPos.x += dx;
        worldPos.y += dy;
        worldPos.heading += -1 * dtheta;

        dashboard.sendTelemetryPacket(packet);

        prev_ticks_back = backRight.getCurrentPosition();
        prev_ticks_left = frontLeft.getCurrentPosition();
        prev_ticks_right = frontRight.getCurrentPosition();
    }

    /**
     * Moves the robot to a specified position
     * @param targetPos Target x, y, and heading for the robot. Heading refers to the preferred angle of the robot
     * @param movementSpeed Motor power
     * @param turnSpeed Robot turn speed
     */
    public void goToPosition(Pose targetPos, double movementSpeed, double turnSpeed) {
        double distanceToTarget = Math.hypot(targetPos.x - worldPos.x, targetPos.y - worldPos.y);

        double absoluteAngleToTarget = Math.atan2(targetPos.y - worldPos.y, targetPos.x - worldPos.x);

        double relativeAngleToTarget = AngleWrap(absoluteAngleToTarget - worldPos.heading - Math.toRadians(90));

        double relativeXToTarget = Math.cos(relativeAngleToTarget) * distanceToTarget;
        double relativeYToTarget = Math.sin(relativeAngleToTarget) * distanceToTarget;
        double relativeTurnAngle = relativeAngleToTarget - Math.toRadians(180) + targetPos.heading;

        // Preserve shape of vector (basically normalize)
        double movementXPower = relativeXToTarget / (Math.abs(relativeXToTarget) + Math.abs(relativeYToTarget)) * movementSpeed;
        double movementYPower = relativeYToTarget / (Math.abs(relativeYToTarget) + Math.abs(relativeXToTarget)) * movementSpeed;
        double turnPower = Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1) * turnSpeed;

        if (distanceToTarget < 10) turnPower = 0;

        frontLeft.setPower(movementXPower - turnPower + movementYPower);
        frontRight.setPower(movementXPower + turnPower - movementYPower);
        backLeft.setPower(movementXPower - turnPower - movementYPower);
        backRight.setPower(movementXPower + turnPower + movementYPower);
    }
}
