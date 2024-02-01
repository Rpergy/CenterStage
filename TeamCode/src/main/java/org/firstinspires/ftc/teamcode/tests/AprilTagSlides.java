package org.firstinspires.ftc.teamcode.tests;

import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.opencv.core.Mat;

import java.util.ArrayList;

public class AprilTagSlides extends OpMode {
    private AprilTagProcessor aprilTag;

    @Override
    public void init() {
        aprilTag = new AprilTagProcessor() {
            @Override
            public void setDecimation(float decimation) {

            }

            @Override
            public void setPoseSolver(PoseSolver poseSolver) {

            }

            @Override
            public int getPerTagAvgPoseSolveTime() {
                return 0;
            }

            @Override
            public ArrayList<AprilTagDetection> getDetections() {
                return null;
            }

            @Override
            public ArrayList<AprilTagDetection> getFreshDetections() {
                return null;
            }

            @Override
            public void init(int width, int height, CameraCalibration calibration) {

            }

            @Override
            public Object processFrame(Mat frame, long captureTimeNanos) {
                return null;
            }

            @Override
            public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

            }
        };
    }

    @Override
    public void loop() {
        for(AprilTagDetection detection : aprilTag.getDetections()) {
            Orientation rot = Orientation.getOrientation(detection.rawPose.R, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            telemetry.addData("Detection " + detection.id, Math.hypot(detection.ftcPose.x, detection.ftcPose.y));
        }
    }
}
