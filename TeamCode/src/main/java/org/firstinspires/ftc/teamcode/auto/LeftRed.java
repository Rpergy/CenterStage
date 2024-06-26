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

import java.util.ArrayList;

@Autonomous(name="left red", group = "red auto")
public class LeftRed extends LinearOpMode {
    OpenCvWebcam webcam;
    double left = 0.0;
    double middle = 0.0;
    double right = 0.0;

    boolean parkLeft = true;
    boolean center = false;

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
            if(gamepad1.right_trigger > 0.5) parkLeft = false;
            else if(gamepad1.left_trigger > 0.5) parkLeft = true;

            if(gamepad1.left_bumper) center = false;
            if(gamepad1.right_bumper) center = true;

            if (Math.max(Math.max(left-0.3, right-0.3), middle-0.2) == middle-0.2) {
                telemetry.addData("prop", "middle");
            }
            else if (Math.max(Math.max(left-0.3, right-0.3), middle-0.2) == left-0.3) {
                telemetry.addData("prop", "left");
            }
            else if (Math.max(Math.max(left, right), middle) == right) {
                telemetry.addData("prop", "right");
            }

            if(parkLeft) telemetry.addData("park", "left");
            else telemetry.addData("park", "right");

            if(center) telemetry.addData("movement", "center");
            else telemetry.addData("movement", "side");
            telemetry.update();
        }

        waitForStart();

        Trajectory start_spike = new Trajectory(FieldConstants.Red.Left.start);

        Trajectory stack_canvas_side = new Trajectory()
                .lineTo(new Pose(-53, -59.75, Math.toRadians(0)))
                .lineTo(new Pose(29, -59.75, Math.toRadians(0)), 0.8, 0.2);

        Trajectory stack_canvas_mid = new Trajectory();

        Trajectory park_right = new Trajectory()
                .lineTo(new Pose(47, -60, Math.toRadians(0)))
                .lineTo(FieldConstants.Red.Park.right);

        Trajectory park_left = new Trajectory()
                .lineTo(new Pose(47, -12, Math.toRadians(0)))
                .lineTo(FieldConstants.Red.Park.left);

        if (Math.max(Math.max(left-0.3, right-0.3), middle-0.2) == middle-0.2) { // CENTER
            start_spike.lineTo(FieldConstants.Red.Left.transition)
                    .lineTo(FieldConstants.Red.Left.centerSpike) // deposit purple
                    .lineTo(new Pose(-36.5, -40, Math.toRadians(0))); // move back

            stack_canvas_side.lineTo(FieldConstants.Red.Canvas.center);
            stack_canvas_mid
                    .lineTo(new Pose(-55, -40, Math.toRadians(90)))
                    .lineTo(new Pose(-55, -12, Math.toRadians(90)))
                    .lineTo(new Pose(40, -12, Math.toRadians(0)))
                    .lineTo(FieldConstants.Red.Canvas.center);
        }
        else if (Math.max(Math.max(left-0.3, right-0.3), middle-0.2) == left-0.3) { // LEFT
            start_spike.lineTo(FieldConstants.Red.Left.transition)
                    .lineTo(FieldConstants.Red.Left.leftSpike) // deposit purple
                    .lineTo(new Pose(-39, -44, Math.toRadians(90))); // move back

            stack_canvas_side.lineTo(FieldConstants.Red.Canvas.left);
            stack_canvas_mid
                    .lineTo(new Pose(-35, -44, Math.toRadians(90)))
                    .lineTo(new Pose(-35, -12, Math.toRadians(90)))
                    .lineTo(new Pose(40, -12, Math.toRadians(0)))
                    .lineTo(FieldConstants.Red.Canvas.left);
        }
        else if (Math.max(Math.max(left, right), middle) == right) { // RIGHT
            start_spike.lineTo(new Pose(-38, -43, Math.toRadians(0)))
                    .lineTo(FieldConstants.Red.Left.rightSpike); // deposit purple

            stack_canvas_side.lineTo(FieldConstants.Red.Canvas.right);
            stack_canvas_mid
                    .lineTo(new Pose(-38, -12, Math.toRadians(0)))
                    .lineTo(new Pose(40, -12, Math.toRadians(0)))
                    .lineTo(FieldConstants.Red.Canvas.right);
        }

        Trajectory deposit_movement = new Trajectory().action(() -> sleep(250))
                .action(Actuation::canvasAlign)
                .action(() -> Actuation.setTilt(ActuationConstants.Extension.tiltPositions[1])) // tilt slides
                .action(Actuation::slidesOut) // send slides out
                .action(() -> sleep(500))
                .action(() -> Actuation.setDepositTilt(ActuationConstants.Deposit.depositTilts[0])) // setup depositor
                .action(() -> sleep(1250))
                .action(() -> Actuation.setDeposit(-1.0)) // start depositor
                .action(() -> sleep(1000))
                .action(() -> Actuation.setDeposit(0.0)) // stop depositor
                .action(() -> sleep(1000))
                .action(()-> Actuation.setDepositTilt(ActuationConstants.Deposit.intakeTilt)) // set depositor
                .action(Actuation::slidesIn) // send slides in
                .action(() -> sleep(1000))
                .action(() -> Actuation.setTilt(ActuationConstants.Extension.tiltPositions[0])); // tilt slides

        start_spike.run();
        //sleep(10000);
        /*if(center)
            stack_canvas_mid.run();
        else
            stack_canvas_side.run();

        deposit_movement.run();

        if(parkLeft)
            park_left.run();
        else
            park_right.run();

        Trajectory planeSetup = new Trajectory()
                .action(() -> Actuation.setPlaneTilt(0.21));

        planeSetup.run();*/

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