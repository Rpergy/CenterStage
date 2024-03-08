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

@Autonomous(name="right red", group = "red auto")
public class RightRed extends LinearOpMode {
    OpenCvWebcam webcam;
    double left = 0;
    double middle = 0;
    double right = 0;
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);
        Actuation.setDeposit(ActuationConstants.Deposit.closed);

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

        Trajectory start_spike = new Trajectory(FieldConstants.Red.Right.start);

        Trajectory spike_canvas = new Trajectory();

        Trajectory park_right = new Trajectory()
                .lineTo(new Pose(42, -60, Math.toRadians(0)))
                .lineTo(FieldConstants.Red.Park.right);

        Trajectory park_left = new Trajectory()
                .lineTo(new Pose(42, -12, Math.toRadians(0)))
                .lineTo(FieldConstants.Red.Park.left);

        if (Math.max(Math.max(left-0.3, right-0.3), middle-0.2) == middle-0.2) { // CENTER
            start_spike.lineTo(FieldConstants.Red.Right.transition)
                    .lineTo(FieldConstants.Red.Right.centerSpike)
                    .lineTo(new Pose(12, -40, Math.toRadians(0)));

            spike_canvas.lineTo(FieldConstants.Red.Canvas.center);
        }
        else if (Math.max(Math.max(left-0.3, right-0.3), middle-0.2) == left-0.3) { // LEFT
            start_spike.lineTo(new Pose(13.5, -36, Math.toRadians(-180)))
                    .lineTo(FieldConstants.Red.Right.leftSpike)
                    .lineTo(new Pose(13.5, -42, Math.toRadians(0)));

            spike_canvas.lineTo(FieldConstants.Red.Canvas.left);
        }
        else if (Math.max(Math.max(left, right), middle) == right) { // RIGHT
            start_spike.lineTo(FieldConstants.Red.Right.transition)
                    .lineTo(FieldConstants.Red.Right.rightSpike)
                    .lineTo(new Pose(23, -50, Math.toRadians(90)));

            spike_canvas.lineTo(FieldConstants.Red.Canvas.right);
        }

        spike_canvas.action(() -> sleep(500))
                .action(Actuation::canvasAlign)
                .action(() -> Actuation.setTilt(ActuationConstants.Extension.tiltPositions[1])) // tilt slides
                .action(Actuation::slidesOut) // send slides out
                .action(() -> sleep(1000))
                .action(() -> Actuation.setDepositTilt(ActuationConstants.Deposit.depositTilts[0])) // setup depositor
                .action(() -> sleep(500))
                .action(() -> Actuation.setDeposit(1.0)) // start depositor
                .action(() -> sleep(1000))
                .action(() -> Actuation.setDeposit(0.0)) // stop depositor
                .action(() -> sleep(500))
                .action(() -> Actuation.setSlides((int)((Actuation.getDist()+0.5) * 125 + 650)));

        if (Math.max(Math.max(left-0.3, right-0.3), middle-0.2) == middle-0.2) { // CENTER
            spike_canvas.lineTo(new Pose(47.5, -39, Math.toRadians(0)));
        }
        else if (Math.max(Math.max(left-0.3, right-0.3), middle-0.2) == left-0.3) { // LEFT
            spike_canvas.lineTo(new Pose(47.5, -31, Math.toRadians(0)));
        }
        else if (Math.max(Math.max(left, right), middle) == right) { // RIGHT
            spike_canvas.lineTo(new Pose(47.5, -42.5, Math.toRadians(0)));
        }

        spike_canvas.action(()-> Actuation.setDepositTilt(ActuationConstants.Deposit.intakeTilt)) // set depositor
                .action(Actuation::slidesIn) // send slides in
                .action(() -> sleep(1000))
                .action(() -> Actuation.setTilt(ActuationConstants.Extension.tiltPositions[0])); // tilt slides

        Trajectory canvas_stack_mid = new Trajectory();
        Trajectory canvas_stack_side = new Trajectory();
        Trajectory stack_canvas_mid = new Trajectory();
        Trajectory stack_canvas_side = new Trajectory();

        start_spike.run();
        spike_canvas.run();
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
            Core.inRange(hsv_left, new Scalar(0, 100, 100), new Scalar(255, 255, 255), hsv_left);
            Core.inRange(hsv_right, new Scalar(0, 100, 100), new Scalar(255, 255, 255), hsv_right);
            Core.inRange(hsv_mid, new Scalar(0, 100, 100), new Scalar(255, 255, 255), hsv_mid);
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