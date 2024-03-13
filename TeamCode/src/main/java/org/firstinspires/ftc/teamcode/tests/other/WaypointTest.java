package org.firstinspires.ftc.teamcode.tests.other;

import java.util.ArrayList;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utility.autonomous.RobotMovement;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@TeleOp(name = "WaypointTest", group = "tests")
@Config
public class WaypointTest extends OpMode {
    private static final int INCHES_PER_QUERY = 1;
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private ArrayList<Point> pursuitPoses;
    private RobotMovement robot;
    FtcDashboard dashboard;

    @Override
    public void init() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");

        dashboard = FtcDashboard.getInstance();

        pursuitPoses = new ArrayList<Point>();
    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            pathTo(50, 50);
        }
        if (gamepad1.x && pursuitPoses.size() > 0) {
            pursuitPoses = new ArrayList<Point>();
        }

        // check for arraylist of purepursuit values and if empty continue
        // need the followingPath toggle?

        double move = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;

        if (pursuitPoses.size() == 0) {
            frontLeft.setPower(move+strafe+turn);
            backLeft.setPower(move-strafe+turn);
            frontRight.setPower(move-strafe-turn);
            backRight.setPower(move+strafe-turn);
        } else {
            frontLeft.setPower(0.25 * (move+strafe+turn));
            backLeft.setPower(0.25 * (move-strafe+turn));
            frontRight.setPower(0.25 * (move-strafe-turn));
            backRight.setPower(0.25 * (move+strafe-turn));
        }

        telemetry.addData("Pathing", pursuitPoses.size() == 0);
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();

        dashboard.sendTelemetryPacket(packet);
    }

    public void pathTo(int row, int col) {
        int[] target = {row, col};
        pursuitPoses = new ArrayList<Point>();
        // button for each loc; dpad?
        // need to acquire current location as a Point / int[]
        // convert code to Point based?

        int rows = 144/INCHES_PER_QUERY;
        boolean[][] occupancyGrid = new boolean[rows][rows];
        for (int r = 0; r < occupancyGrid.length; r++) {
            for (int c = 0; c < occupancyGrid[r].length; c++) {
                occupancyGrid[r][c] = getOccupancy(r, c);
            }
        }

        // get current loc
        int[] TEMPORARY_CURRENT_LOCATION = {0, 0};

        pursuitPoses.addAll(shortestPath(occupancyGrid, TEMPORARY_CURRENT_LOCATION, target));

        // get path coords and add to arraylist of pathing
        // need to make pure pursuit work within teleop while also giving some (0.25x or smthg) control to drivers
    }

    public boolean getOccupancy(int row, int col) {
        // should be obtained from occupancy map
        return false;
    }

    public ArrayList<Point> shortestPath(boolean[][] grid, int[] start, int[] end) {
        if (!grid[start[0]][start[1]] || !grid[end[0]][end[1]]) {
            return null;
        }

        ArrayList<int[]> queue = new ArrayList<int[]>();
        boolean[][] visited = new boolean[grid.length][grid[0].length];
        visited[start[0]][start[1]] = true;
        int[][][] parent = new int[grid.length][grid[0].length][2];
        for (int r = 0; r < parent.length; r++) {
            for (int c = 0; c < parent[r].length; c++) {
                parent[r][c][0] = -1;
                parent[r][c][1] = -1;
            }
        }

        queue.add(start);

        while (queue.size() > 0) {
            int[] current = queue.remove(0);
            int currentRow = current[0];
            int currentCol = current[1];

            if (currentRow == end[0] && currentCol == end[1]) {
                return reconstructPath(parent, currentRow, currentCol, start);
            }

            int[][] offsets = {{-1, 0}, {1, 0}, {0, 1}, {0, -1}};
            for (int i = 0; i < offsets.length; i++) {
                int nextRow = currentRow + offsets[i][0];
                int nextCol = currentCol + offsets[i][1];
                if (nextRow >= 0 && nextRow < grid.length && nextCol >= 0 && nextCol < grid[0].length) {
                    if (!visited[nextRow][nextCol] && grid[nextRow][nextCol]) {
                        visited[nextRow][nextCol] = true;
                        parent[nextRow][nextCol] = current;
                        int[] nextCoord = {nextRow, nextCol};
                        queue.add(nextCoord);
                    }
                }
            }
        }

        System.out.println("No path exists.");
        return null;
    }

    public ArrayList<Point> reconstructPath(int[][][] parent, int endRow, int endCol, int[] start) {
        ArrayList<Point> path = new ArrayList<Point>();
        while (parent[endRow][endCol][0] != -1) {
            int[] endCoord = {endRow, endCol};
            path.add(0, new Point(endCoord));
            int prevRow = endRow;
            int prevCol = endCol;
            endRow = parent[prevRow][prevCol][0];
            endCol = parent[prevRow][prevCol][1];
        }
        path.add(0, new Point(start));

        return path;
    }
}
