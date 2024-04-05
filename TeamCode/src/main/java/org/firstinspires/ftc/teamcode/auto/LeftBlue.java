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

@Autonomous(name="left blue", group = "blue auto")
public class LeftBlue extends LinearOpMode {
    OpenCvWebcam webcam;
    double left = 1;
    double middle = 0;
    double right = 0;
    boolean parkLeft = true;
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

            if (Math.max(Math.max(left, right), middle) == middle) {
                telemetry.addData("prop", "middle");
            }
            else if (Math.max(Math.max(left, right), middle) == left) {
                telemetry.addData("prop", "left");
            }
            else if (Math.max(Math.max(left, right), middle) == right) {
                telemetry.addData("prop", "right");
            }
            telemetry.update();
        }

        waitForStart();
        Trajectory start_spike = new Trajectory(FieldConstants.Blue.Left.start);

        Trajectory spike_canvas = new Trajectory();

        if (Math.max(Math.max(left, right), middle) == middle) { // CENTER
            start_spike.lineTo(FieldConstants.Blue.Left.transition)
                    .lineTo(FieldConstants.Blue.Left.centerSpike)
                    .lineTo(new Pose(11.5, 44, Math.toRadians(-90)));

            spike_canvas.lineTo(FieldConstants.Blue.Canvas.center, 0.6, 0.6);
        }
        else if (Math.max(Math.max(left, right), middle) == left) { // LEFT
            start_spike.lineTo(FieldConstants.Blue.Left.transition)
                    .lineTo(FieldConstants.Blue.Left.leftSpike)
                    .lineTo(new Pose(21, 48, Math.toRadians(0)));

            spike_canvas.lineTo(FieldConstants.Blue.Canvas.left, 0.6, 0.6);
        }
        else if (Math.max(Math.max(left, right), middle) == right) { // RIGHT
            start_spike.lineTo(new Pose(14, 36, Math.toRadians(-180)))
                    .lineTo(FieldConstants.Blue.Left.rightSpike, 0.6, 0.2)
                    .lineTo(new Pose(16, 38, Math.toRadians(-180)));

            spike_canvas.lineTo(FieldConstants.Blue.Canvas.right, 0.6, 0.6);
        }

        spike_canvas.action(() -> sleep(250))
                .action(Actuation::canvasAlign)
                .action(() -> Actuation.setTilt(ActuationConstants.Extension.tiltPositions[1])) // tilt slides
                .action(Actuation::slidesOut) // send slides out
                .action(() -> sleep(500))
                .action(() -> Actuation.setDepositTilt(ActuationConstants.Deposit.depositTilts[0])) // setup depositor
                .action(() -> sleep(1250))
                .action(() -> Actuation.setDeposit(-1.0)) // start depositor
                .action(() -> sleep(1000))
                .action(() -> Actuation.setDeposit(0.0)) // stop depositor
                .action(() -> sleep(1000));

//        if (Math.max(Math.max(left, right), middle) == middle) { // CENTER
//            spike_canvas.lineTo(new Pose(45, 37.25, Math.toRadians(0)));
//        }
//        else if (Math.max(Math.max(left, right), middle) == left) { // LEFT
//            spike_canvas.lineTo(new Pose(45, 44, Math.toRadians(0)));
//        }
//        else if (Math.max(Math.max(left, right), middle) == right) { // RIGHT
//            spike_canvas.lineTo(new Pose(45, 32.5, Math.toRadians(0)));
//        }

        spike_canvas.action(()-> Actuation.setDepositTilt(ActuationConstants.Deposit.intakeTilt)) // set depositor
                .action(Actuation::slidesIn) // send slides in
                .action(() -> sleep(1000))
                .action(() -> Actuation.setTilt(ActuationConstants.Extension.tiltPositions[0])); // tilt slides

        Trajectory canvas_stack_mid = new Trajectory()
                .lineTo(new Pose(37, 10, 0))
                .lineTo(FieldConstants.Blue.Stacks.left, 0.9, 0.5);
        Trajectory canvas_stack_side = new Trajectory();

        Trajectory stack_canvas_mid = new Trajectory()
                .lineTo(new Pose(37, 10, 0), 0.9, 0.5)
                .lineTo(FieldConstants.Blue.Canvas.center);
        Trajectory stack_canvas_side = new Trajectory();

        Trajectory park_left = new Trajectory()
                .lineTo(new Pose(45, 61, Math.toRadians(0)))
                .lineTo(FieldConstants.Blue.Park.left);

        Trajectory park_right = new Trajectory()
                .lineTo(new Pose(45, 11, 0))
                .lineTo(FieldConstants.Blue.Park.right);

        start_spike.run();
        spike_canvas.run();

        /*Trajectory goOver = new Trajectory()
                .lineTo(new Pose(40, 14, Math.toRadians(0)))
                .lineTo(new Pose(-45, 14, Math.toRadians(0)))
                .lineTo(new Pose(-50.5, 14, Math.toRadians(0)), 0.5, 0.5);

        Actuation.setIntakeArm(ActuationConstants.Intake.stackPos[0]);
        Actuation.setIntakeArm(ActuationConstants.Intake.stackPos[5]);

        goOver.run();
        Actuation.setIntakeArm(ActuationConstants.Intake.stackPos[3]);
        Trajectory goBack = new Trajectory()
                .lineTo(new Pose(-45, 14, Math.toRadians(0)), 0.5, 0.5)
                .lineTo(new Pose(-49, 14, Math.toRadians(0)), 0.5, 0.5);
        goBack.run();
        double start = System.currentTimeMillis();
        while(System.currentTimeMillis()-start<5000) {
            Actuation.setIntakeArm(ActuationConstants.Intake.stackPos[1]);
            if (Actuation.getColorTop()[0] >= 1055 && Actuation.getColorTop()[0]<= 1155 && Actuation.getColorTop()[1]>= 2150 && Actuation.getColorTop()[1]<= 2250) {
                Actuation.setIntake(-1); // Intake
                Actuation.toggleDeposit(1);
            }
            else {
                Actuation.setIntake(1); // Extake
            }
        }
        //Actuation.setIntake(0);
        Trajectory retreat = new Trajectory()
                .lineTo(new Pose(40, 14, Math.toRadians(0)), 0.5, 0.5)
                .lineTo(FieldConstants.Blue.Canvas.right);
        retreat.run();
        spike_canvas.run();*/


        if(parkLeft)
            park_left.run();
        else
            park_right.run();
    }
    class Pipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input)
        {
            Mat hsv = input.clone();

            Imgproc.cvtColor(hsv, hsv, Imgproc.COLOR_RGB2HSV);

            Core.inRange(hsv, new Scalar(100, 0, 100), new Scalar(255, 255, 255), hsv);

            // debug drawing for finding where block spaces are
            Imgproc.rectangle(hsv, new Point(0, 110), new Point(50, 160), new Scalar(255, 255, 0));
            Imgproc.rectangle(hsv, new Point(280, 110), new Point(320, 160), new Scalar(255, 255, 0));
            Imgproc.rectangle(hsv, new Point(100, 110), new Point(250, 140), new Scalar(255, 255, 0));

            Mat hsv_left = new Mat();
            Mat hsv_right = new Mat();
            Mat hsv_mid = new Mat();

            Mat left_sub = input.submat(new Range(110, 160), new Range(0, 50));
            Mat right_sub = input.submat(new Range(110, 160), new Range(280, 320));
            Mat mid_sub = input.submat(new Range(110, 140), new Range(100, 250));

            Imgproc.cvtColor(right_sub, hsv_right, Imgproc.COLOR_RGB2HSV);
            Imgproc.cvtColor(left_sub, hsv_left, Imgproc.COLOR_RGB2HSV);
            Imgproc.cvtColor(mid_sub, hsv_mid, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsv_left, new Scalar(100, 0, 100), new Scalar(255, 255, 255), hsv_left);
            Core.inRange(hsv_right, new Scalar(100, 0, 100), new Scalar(255, 255, 255), hsv_right);
            Core.inRange(hsv_mid, new Scalar(100, 0, 100), new Scalar(255, 255, 255), hsv_mid);

            Imgproc.rectangle(input, new Point(0, 110), new Point(50, 160), new Scalar(255, 255, 0));
            Imgproc.rectangle(input, new Point(280, 110), new Point(320, 160), new Scalar(255, 255, 0));
            Imgproc.rectangle(input, new Point(100, 110), new Point(250, 140), new Scalar(255, 255, 0));

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