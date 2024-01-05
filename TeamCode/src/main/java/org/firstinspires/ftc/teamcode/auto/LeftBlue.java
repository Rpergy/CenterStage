package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.RobotMovement;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;

@Autonomous(name="left blue")
public class LeftBlue extends LinearOpMode {
//    OpenCvWebcam webcam;
    double left = 0;
    double middle = 0;
    double right = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Actuation.setup(hardwareMap);
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
        RobotMovement robot = new RobotMovement(hardwareMap, new Pose(11.5, 63, Math.toRadians(-90)));
//
//        I think there's an issue where if you command the robot to turn more than it can in the path length is doesn't make the full rotation(?)
        ArrayList<Pose> startToRight = new ArrayList<>();
        startToRight.add(new Pose(11.5, 63, Math.toRadians(-90)));
        startToRight.add(new Pose(11.5, 63, Math.toRadians(-180)));

        ArrayList<Pose> rightToCanvas = new ArrayList<>();
        rightToCanvas.add(new Pose(9, 36, Math.toRadians(-180)));
        rightToCanvas.add(new Pose(49.8, 36, Math.toRadians(-180)));

//        robot.displayPoses(startToRight, ActuationConstants.Autonomous.followDistance);
//        robot.displayPoses(rightToCanvas, ActuationConstants.Autonomous.followDistance);

//        ArrayList<Pose> startToCanvas = new ArrayList<>();
//        startToCanvas.add(new Pose(11.5, 63, Math.toRadians(-90)));
//        startToCanvas.add(new Pose(36, 48, Math.toRadians(-135)));
//        startToCanvas.add(new Pose(49.8, 36, Math.toRadians(-180)));

//        ArrayList<Pose> canvasToStacks = new ArrayList<>();
//        canvasToStacks.add(new Pose(49.8, 36, Math.toRadians(-180)));
//        canvasToStacks.add(new Pose(0, 17, Math.toRadians(-180)));
//        canvasToStacks.add(new Pose(-40, 17, Math.toRadians(-180)));
//        canvasToStacks.add(new Pose(-55, 27.65, Math.toRadians(-180)));
//        canvasToStacks.add(new Pose(-63, 27.65, Math.toRadians(-180)));
//
//        ArrayList<Pose> stacksToMid = new ArrayList<>();
//        stacksToMid.add(new Pose(-63, 27.6, Math.toRadians(-180)));
//        stacksToMid.add(new Pose(-55, 27.6, Math.toRadians(-180)));
//        stacksToMid.add(new Pose(-40, 17, Math.toRadians(-180)));
//        stacksToMid.add(new Pose(10, 17, Math.toRadians(-180)));
//
//        ArrayList<Pose> midToCanvas = new ArrayList<>();
//        midToCanvas.add(new Pose(10, 17, Math.toRadians(-180)));
//        midToCanvas.add(new Pose(50, 36, Math.toRadians(-180)));
//

        FtcDashboard dashboard = FtcDashboard.getInstance();
        waitForStart();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("deltaTheta", 0);
        dashboard.sendTelemetryPacket(packet);

        sleep(5000);

        while(true) {
            robot.goToPose(new Pose(11.5, 63, Math.toRadians(-180)), ActuationConstants.Autonomous.moveSpeed, ActuationConstants.Autonomous.turnSpeed);
            robot.updatePosition();
        }

//        robot.followPoseCurve(telemetry, startToRight, ActuationConstants.Autonomous.followDistance, ActuationConstants.Autonomous.moveSpeed, ActuationConstants.Autonomous.turnSpeed);
//        Actuation.setTilt(ActuationConstants.Extension.tiltPresets[3]);
//        Actuation.setWrist(ActuationConstants.Claw.wristIntake);
//        sleep(500);
//        Actuation.setLClaw(ActuationConstants.Claw.open);
//        sleep(200);
//        Actuation.setTilt(0.3);
//        sleep(500);
//
//        Actuation.setWrist(ActuationConstants.Claw.wristDeposit);
//        Actuation.setTilt(ActuationConstants.Extension.tiltPresets[0]);
//        Actuation.setExtension(200);
//        robot.followPoseCurve(telemetry, rightToCanvas, ActuationConstants.Autonomous.followDistance, ActuationConstants.Autonomous.moveSpeed, ActuationConstants.Autonomous.turnSpeed);
//        sleep(1200);
//        Actuation.setClaw(ActuationConstants.Claw.open);
//        sleep(250);

//        Actuation.setWrist(ActuationConstants.Claw.wristDeposit);
//        Actuation.setTilt(ActuationConstants.Extension.tiltPresets[0]);
//        Actuation.setExtension(200);
//        robot.followPoseCurve(telemetry, startToCanvas, ActuationConstants.Autonomous.followDistance, ActuationConstants.Autonomous.moveSpeed, ActuationConstants.Autonomous.turnSpeed);
//        Actuation.setClaw(ActuationConstants.Claw.open);
//        sleep(250);
//
//        Actuation.setExtension(ActuationConstants.Extension.extensionPresets[3]);
//        Actuation.setWrist(ActuationConstants.Claw.wristIntake);
//        Actuation.setTilt(ActuationConstants.Extension.tiltPresets[3] + 0.031);
//        robot.followPoseCurve(telemetry, canvasToStacks, ActuationConstants.Autonomous.followDistance, ActuationConstants.Autonomous.moveSpeed, ActuationConstants.Autonomous.turnSpeed);
//        Actuation.setClaw(ActuationConstants.Claw.closed);
//        sleep(250);
//
//        robot.followPoseCurve(telemetry, stacksToMid, ActuationConstants.Autonomous.followDistance, ActuationConstants.Autonomous.moveSpeed, ActuationConstants.Autonomous.turnSpeed);
//
//        Actuation.setTilt(ActuationConstants.Extension.tiltPresets[0]);
//        Actuation.setWrist(ActuationConstants.Claw.wristDeposit);
//        Actuation.setExtension(200);
//
//        robot.followPoseCurve(telemetry, midToCanvas, ActuationConstants.Autonomous.followDistance, ActuationConstants.Autonomous.moveSpeed, ActuationConstants.Autonomous.turnSpeed);
//        sleep(1000);
//        Actuation.setClaw(ActuationConstants.Claw.open);
//        sleep(5000);
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