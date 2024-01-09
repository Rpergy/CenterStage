package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.RobotMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;

@Autonomous(name="left blue", group = "auto")
public class LeftBlue extends LinearOpMode {
//    OpenCvWebcam webcam;
    double left = 0;
    double middle = 0;
    double right = 0;
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);
        Actuation.setClaw(ActuationConstants.Claw.closed);
        Actuation.setWrist(ActuationConstants.Claw.wristAutoInit);
        Actuation.setTilt(0.3);

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        webcam.setPipeline(new LeftBlue.Pipeline());
//        webcam.setMillisecondsPermissionTimeout(5000);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            public void onError(int errorCode) {}
//        });
//
//        if (left > 0.3) {
//            telemetry.addData("left", true);
//        } else {
//            telemetry.addData("left", false);
//        }
//        if (right > 0.3) {
//            telemetry.addData("right", true);
//        } else {
//            telemetry.addData("right", false);
//        }
//        if (middle > 0.2) {
//            telemetry.addData("middle", true);
//        } else {
//            telemetry.addData("middle", false);
//        }
//        telemetry.update();

//        commented because testing vision code
        waitForStart();
        Trajectory leftBlue = new Trajectory(new Pose(11.5, 63, Math.toRadians(-90)))
                .action(() -> Actuation.setTilt(ActuationConstants.Extension.tiltPresets[0]))
                .action(() -> Actuation.setWrist(ActuationConstants.Claw.wristDeposit))
                .lineTo(new Pose(50, 34.5, Math.toRadians(180)))
                .action(() -> Actuation.setLClaw(ActuationConstants.Claw.open))
                .action(() -> sleep(500))
                .action(() -> Actuation.setTilt(ActuationConstants.Extension.tiltPresets[3]))
                .action(() -> Actuation.setWrist(ActuationConstants.Claw.wristIntake))
                .lineTo(new Pose(22, 28, Math.toRadians(180)), 0.3)
                .action(() -> Actuation.setRClaw(ActuationConstants.Claw.open))
                .action(() -> sleep(500))
                .lineTo(new Pose(28, 28, Math.toRadians(180)))
                .lineTo(new Pose(0, 6, Math.toRadians(180)))
                .lineTo(new Pose(54, 12, Math.toRadians(180)));
    }
//    class Pipeline extends OpenCvPipeline
//    {
//        boolean viewportPaused;
//
//        @Override
//        public Mat processFrame(Mat input)
//        {
//            // debug drawing for finding where block spaces are
////            Imgproc.rectangle(input, new Point(0, 80), new Point(50, 160), new Scalar(255, 255, 0));
////            Imgproc.rectangle(input, new Point(280, 80), new Point(320, 160), new Scalar(255, 255, 0));
////            Imgproc.rectangle(input, new Point(100, 80), new Point(250, 140), new Scalar(255, 255, 0));
//
//            Mat hsv_left = new Mat();
//            Mat hsv_right = new Mat();
//            Mat hsv_mid = new Mat();
//
//            Mat left_sub = input.submat(new Range(80, 160), new Range(0, 50));
//            Mat right_sub = input.submat(new Range(80, 160), new Range(280, 320));
//            Mat mid_sub = input.submat(new Range(80, 140), new Range(100, 250));
//
//            Imgproc.cvtColor(right_sub, hsv_right, Imgproc.COLOR_RGB2HSV);
//            Imgproc.cvtColor(left_sub, hsv_left, Imgproc.COLOR_RGB2HSV);
//            Imgproc.cvtColor(mid_sub, hsv_mid, Imgproc.COLOR_RGB2HSV);
//            Core.inRange(hsv_left, new Scalar(100, 0, 100), new Scalar(255, 255, 255), hsv_left);
//            Core.inRange(hsv_right, new Scalar(100, 0, 100), new Scalar(255, 255, 255), hsv_right);
//            Core.inRange(hsv_mid, new Scalar(100, 0, 100), new Scalar(255, 255, 255), hsv_mid);
//            left = Core.sumElems(hsv_left).val[0]/(80*50*255);
//            right = Core.sumElems(hsv_right).val[0]/(80*40*255);
//            middle = Core.sumElems(hsv_mid).val[0]/(60*150*255);
//            return input;
//        }
//
//        @Override
//        public void onViewportTapped()
//        {
//            viewportPaused = !viewportPaused;
//
//            if(viewportPaused)
//            {
//                webcam.pauseViewport();
//            }
//            else
//            {
//                webcam.resumeViewport();
//            }
//        }
//    }
}