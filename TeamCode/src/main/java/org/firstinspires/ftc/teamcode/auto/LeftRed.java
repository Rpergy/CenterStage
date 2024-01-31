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

@Autonomous(name="left red", group = "auto")
public class LeftRed extends LinearOpMode {
    OpenCvWebcam webcam;
    double left = 1.0;
    double middle = 0;
    double right = 0;
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);
        Actuation.setClaw(ActuationConstants.Claw.closed);
        Actuation.setWrist(ActuationConstants.Claw.wristAutoInit);
        Actuation.setTilt(0.3);

        left = 0.0;
        right = 1.0;
        middle = 0.0;

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        webcam.setPipeline(new Pipeline());
//        webcam.setMillisecondsPermissionTimeout(5000);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            public void onError(int errorCode) {}
//        });

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

//        commented because testing vision code
        waitForStart();

        Trajectory start_spike = new Trajectory(FieldConstants.Red.Left.start)
                .lineTo(FieldConstants.Red.Left.transition);

        if (Math.max(Math.max(left-0.3, right-0.3), middle-0.2) == middle-0.2) { // CENTER
            start_spike.lineTo(FieldConstants.Red.Left.centerSpike)
                    .lineTo(new Pose(-36.5, -40, Math.toRadians(0)));
        }
        else if (Math.max(Math.max(left-0.3, right-0.3), middle-0.2) == left-0.3) { // LEFT
            start_spike.lineTo(FieldConstants.Red.Left.leftSpike)
                    .lineTo(new Pose(-46.5, -44, Math.toRadians(0)));
        }
        else if (Math.max(Math.max(left, right), middle) == right) { // RIGHT
            start_spike.lineTo(FieldConstants.Red.Left.rightSpike)
                    .lineTo(new Pose(-34, -42, Math.toRadians(0)));
        }

        Trajectory farStack_Canvas = new Trajectory()
                .lineTo(new Pose(-40, 14, Math.toRadians(180)))
                .lineTo(new Pose(30, 14, Math.toRadians(180)), 0.5, ActuationConstants.Autonomous.turnSpeed)
                .action(() -> Actuation.setTilt(ActuationConstants.Extension.tiltPresets[0]))
                .action(() -> Actuation.setWrist(ActuationConstants.Claw.wristDeposit))
                .lineTo(new Pose(43, 34.5, Math.toRadians(180)))
                .lineTo(FieldConstants.Blue.Canvas.center)
                .action(() -> sleep(1500))
                .action(() -> Actuation.setClaw(ActuationConstants.Claw.open))
                .action(() -> sleep(400));

        Trajectory canvas_park = new Trajectory()
                .action(() -> Actuation.setTilt(ActuationConstants.Extension.tiltPresets[3]))
                .action(() -> Actuation.setWrist(ActuationConstants.Claw.wristIntake))
                .lineTo(new Pose(47, 10, Math.toRadians(180)))
                .lineTo(new Pose(54, 10, Math.toRadians(180)));

        start_spike.run();

//        canvas_farStack.run();
//        farStack_Canvas.run();
//        canvas_park.run();
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