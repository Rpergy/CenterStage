package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.RobotMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Range;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.lang.reflect.Field;
import java.util.ArrayList;

@Autonomous(name="right blue", group = "blue auto")
public class RightBlue extends LinearOpMode {
    OpenCvWebcam webcam;
    double left = 0;
    double middle = 0;
    double right = 0;
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new Pipeline());
        webcam.setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {}
        });

        while(opModeInInit()) {

            if (Math.max(Math.max(left-0.3, right-0.3), middle-0.2) == middle-0.2) {
                telemetry.addData("prop", "middle");
            }
            else if (Math.max(Math.max(left-0.3, right-0.3), middle-0.2) == left-0.3) {
                telemetry.addData("prop", "left");
            }
            else if (Math.max(Math.max(left, right), middle) == right) {
                telemetry.addData("prop", "right");
            }
            telemetry.update();
        }

        waitForStart();

        Trajectory start_spike = new Trajectory(FieldConstants.Blue.Right.start);

        Trajectory spike_stack = new Trajectory(); // to pixel stack
//                .action(() -> Actuation.setIntakeArm(ActuationConstants.Intake.stackPos[5]))
//                .lineTo(FieldConstants.Blue.Stacks.right, 0.6, 0.7)
//                .action(() -> sleep(500))
//                .action(() -> Actuation.setIntake(-1))
//                .action(() -> sleep(500))
//                .action(() -> Actuation.setIntakeArm(ActuationConstants.Intake.stackPos[4]))
//                .action(() -> sleep(500))
//                .lineTo(new Pose(-57, 37, Math.toRadians(0)))
//                .action(() -> sleep(1000));

        Trajectory stack_canvas_side = new Trajectory()
                .lineTo(new Pose(-58, 59, Math.toRadians(0))) // line up with truss
                .lineTo(new Pose(28, 59, Math.toRadians(0)), 0.8, 0.8); // move to blue left

        Trajectory stack_canvas_mid = new Trajectory() // DOES NOT WORK
                .lineTo(new Pose(40, 42, 0), 0.9, 0.5)
                .lineTo(new Pose(37, 10, 0), 0.9, 0.5);

        Trajectory canvas_stack_mid = new Trajectory() // DOES NOT WORK
                .lineTo(new Pose(37, 10, 0), 0.9, 0.5)
                .lineTo(new Pose(40, 42, 0), 0.9, 0.5)
                .lineTo(FieldConstants.Blue.Stacks.right);

        Trajectory canvas_stack_side = new Trajectory()
                .lineTo(new Pose(30, 58.75, Math.toRadians(0))) // line up with truss
                .lineTo(new Pose(-58, 58.75, Math.toRadians(0)), 0.8, 0.8) // move close to stack
                .lineTo(FieldConstants.Blue.Stacks.right); // into stack

        Trajectory park_left = new Trajectory()
                .lineTo(new Pose(43, 59, Math.toRadians(0)))
                .lineTo(FieldConstants.Blue.Park.left);

        Trajectory park_right = new Trajectory()
                .lineTo(new Pose(43, 13, 0))
                .lineTo(FieldConstants.Blue.Park.right);

        if (Math.max(Math.max(left-0.3, right-0.3), middle-0.2) == middle-0.2) { // CENTER
            start_spike.lineTo(FieldConstants.Blue.Right.transition)
                    .lineTo(FieldConstants.Blue.Right.centerSpike) // deposit purple
                    .lineTo(FieldConstants.Blue.Right.transition); // move back

            // move into canvas pos
            stack_canvas_side.lineTo(FieldConstants.Blue.Canvas.center);
            stack_canvas_mid.lineTo(FieldConstants.Blue.Canvas.center);
        }
        else if (Math.max(Math.max(left-0.3, right-0.3), middle-0.2) == left-0.3) { // LEFT
            start_spike.lineTo(new Pose(-40, 38, Math.toRadians(0)), 0.7, 0.8)
                    .lineTo(FieldConstants.Blue.Right.leftSpike)
                    .lineTo(new Pose(-40, 37, 0)); // deposit purple

            // move into canvas pos
            stack_canvas_side.lineTo(FieldConstants.Blue.Canvas.left);
            stack_canvas_mid.lineTo(FieldConstants.Blue.Canvas.left);
        }
        else if (Math.max(Math.max(left, right), middle) == right) { // RIGHT
            start_spike.lineTo(FieldConstants.Blue.Right.transition)
                    .lineTo(FieldConstants.Blue.Right.rightSpike) // deposit purple
                    .lineTo(new Pose(-48.5, 46, Math.toRadians(-90))); // move back

            // move into canvas pos
            stack_canvas_side.lineTo(FieldConstants.Blue.Canvas.right);
            stack_canvas_mid.lineTo(FieldConstants.Blue.Canvas.right);
        }

        stack_canvas_side.action(() -> sleep(250))
                .action(Actuation::canvasAlign)
                .action(() -> Actuation.setTilt(ActuationConstants.Extension.tiltPositions[1])) // tilt slides
                .action(Actuation::slidesOut) // send slides out
                .action(() -> sleep(1500))
                .action(() -> Actuation.setDepositTilt(ActuationConstants.Deposit.depositTilts[0])) // setup depositor
                .action(() -> sleep(500))
                .action(() -> Actuation.setDeposit(ActuationConstants.Deposit.open)) // open depositor
                .action(() -> sleep(1000))
                .action(() -> Actuation.setSlides((int)((Actuation.getDist()+0.5) * 125 + 650)));

        if (Math.max(Math.max(left-0.3, right-0.3), middle-0.2) == middle-0.2) { // CENTER
            stack_canvas_side.lineTo(new Pose(38, 37.25, Math.toRadians(0)));
        }
        else if (Math.max(Math.max(left-0.3, right-0.3), middle-0.2) == left-0.3) { // LEFT
            stack_canvas_side.lineTo(new Pose(38, 44, Math.toRadians(0)));
        }
        else if (Math.max(Math.max(left, right), middle) == right) { // RIGHT
            stack_canvas_side.lineTo(new Pose(38, 32.5, Math.toRadians(0)));
        }

        stack_canvas_side.action(()-> Actuation.setDepositTilt(ActuationConstants.Deposit.intakeTilt)) // set depositor
                .action(Actuation::slidesIn) // send slides in
                .action(() -> sleep(1000))
                .action(() -> Actuation.setTilt(ActuationConstants.Extension.tiltPositions[0])); // tilt slides

        // purple preload
        start_spike.run();
        spike_stack.run();

        // yellow preload and cycle
        stack_canvas_side.run();
//        canvas_stack_side.run();
//        stack_canvas_side.run();

        //park
        park_right.run();
    }
    class Pipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input)
        {
            // debug drawing for finding where block spaces are
            Imgproc.rectangle(input, new Point(0, 80), new Point(50, 160), new Scalar(255, 255, 0));
            Imgproc.rectangle(input, new Point(280, 80), new Point(320, 160), new Scalar(255, 255, 0));
            Imgproc.rectangle(input, new Point(100, 80), new Point(250, 140), new Scalar(255, 255, 0));

            Mat hsv_left = new Mat();
            Mat hsv_right = new Mat();
            Mat hsv_mid = new Mat();

            Mat left_sub = input.submat(new Range(80, 160), new Range(0, 50));
            Mat right_sub = input.submat(new Range(80, 160), new Range(280, 320));
            Mat mid_sub = input.submat(new Range(80, 140), new Range(100, 250));

            Imgproc.cvtColor(right_sub, hsv_right, Imgproc.COLOR_RGB2HSV);
            Imgproc.cvtColor(left_sub, hsv_left, Imgproc.COLOR_RGB2HSV);
            Imgproc.cvtColor(mid_sub, hsv_mid, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsv_left, new Scalar(100, 0, 100), new Scalar(255, 255, 255), hsv_left);
            Core.inRange(hsv_right, new Scalar(100, 0, 100), new Scalar(255, 255, 255), hsv_right);
            Core.inRange(hsv_mid, new Scalar(100, 0, 100), new Scalar(255, 255, 255), hsv_mid);
            left = Core.sumElems(hsv_left).val[0]/(80*50*255);
            right = Core.sumElems(hsv_right).val[0]/(80*40*255);
            middle = Core.sumElems(hsv_mid).val[0]/(60*150*255);
            return input;
        }

        @Override
        public void onViewportTapped()
        {
            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}