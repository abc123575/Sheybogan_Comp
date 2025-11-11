package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;

/**
 * AprilTag + Ball Tracker (Overlap-Aware Version)
 * Handles multiple and overlapping balls using distance transform segmentation.
 */
@Autonomous(name = "AprilTag + Ball Tracker (Overlap-Aware)", group = "Vision")
public class AprilTagAndBallTracker extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private ColorBallProcessor colorProcessor;

    @Override
    public void runOpMode() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        colorProcessor = new ColorBallProcessor();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .addProcessor(colorProcessor)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        telemetry.addLine("✅ Vision initialized — press ▶ to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            int id = detections.isEmpty() ? -1 : detections.get(0).id;

            telemetry.addData("AprilTag ID", id);
            telemetry.addData("Detected Color", colorProcessor.getDetectedColorName());
            telemetry.update();

            sleep(50);
        }

        visionPortal.close();
    }

    /**
     * Processor for purple/green ball detection with overlap handling.
     */
    static class ColorBallProcessor implements VisionProcessor {
        private int detectedColor = 0;
        private double purpleCount = 0;
        private double greenCount = 0;

        @Override
        public void init(int width, int height, CameraCalibration calibration) {}

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Mat hsv = new Mat();
            Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

            Scalar lowerPurple = new Scalar(115, 50, 50);
            Scalar upperPurple = new Scalar(165, 255, 255);
            Scalar lowerGreen  = new Scalar(30, 40, 40);
            Scalar upperGreen  = new Scalar(90, 255, 255);

            Mat purpleMask = new Mat();
            Mat greenMask = new Mat();
            Core.inRange(hsv, lowerPurple, upperPurple, purpleMask);
            Core.inRange(hsv, lowerGreen, upperGreen, greenMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(3, 3));
            Imgproc.morphologyEx(purpleMask, purpleMask, Imgproc.MORPH_CLOSE, kernel);
            Imgproc.morphologyEx(greenMask, greenMask, Imgproc.MORPH_CLOSE, kernel);

            purpleCount = Core.countNonZero(purpleMask);
            greenCount = Core.countNonZero(greenMask);

            // Separate overlapping balls for each color
            segmentAndDetect(frame, purpleMask, new Scalar(255, 0, 255), "PURPLE");
            segmentAndDetect(frame, greenMask, new Scalar(0, 255, 0), "GREEN");

            if (purpleCount > 2000 && purpleCount > greenCount) detectedColor = 1;
            else if (greenCount > 2000 && greenCount > purpleCount) detectedColor = 2;
            else detectedColor = 0;

            Imgproc.putText(frame, "PurpleCount: " + (int)purpleCount, new Point(20,40),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(255,0,255), 2);
            Imgproc.putText(frame, "GreenCount: " + (int)greenCount, new Point(20,70),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(0,255,0), 2);

            hsv.release();
            purpleMask.release();
            greenMask.release();
            kernel.release();
            return frame;
        }

        /**
         * Enhanced detection: uses distance transform to separate overlapping blobs.
         */
        private void segmentAndDetect(Mat frame, Mat mask, Scalar drawColor, String label) {
            // --- Distance transform for splitting ---
            Mat dist = new Mat();
            Imgproc.distanceTransform(mask, dist, Imgproc.DIST_L2, 3);
            Core.normalize(dist, dist, 0, 255, Core.NORM_MINMAX);

            // Threshold to get distinct peaks
            Mat distThresh = new Mat();
            Imgproc.threshold(dist, distThresh, 60, 255, Imgproc.THRESH_BINARY);

            // Convert to 8-bit
            distThresh.convertTo(distThresh, 0);

            // Find contours (each peak roughly one ball)
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(distThresh, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area < 200) continue;

                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                Point center = new Point();
                float[] radius = new float[1];
                Imgproc.minEnclosingCircle(contour2f, center, radius);

                if (radius[0] > 5 && radius[0] < 200) {
                    Imgproc.circle(frame, center, (int) radius[0], drawColor, 3);
                    Imgproc.putText(frame, label,
                            new Point(center.x - 30, center.y - radius[0] - 10),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, drawColor, 2);
                }

                contour2f.release();
            }

            hierarchy.release();
            dist.release();
            distThresh.release();
            for (MatOfPoint c : contours) c.release();
        }

        @Override
        public void onDrawFrame(android.graphics.Canvas canvas,
                                int onscreenWidth,
                                int onscreenHeight,
                                float scaleBmpPxToCanvasPx,
                                float scaleCanvasDensity,
                                Object userContext) {}

        public int getDetectedColor() {
            return detectedColor;
        }

        public String getDetectedColorName() {
            if (detectedColor == 1) return "Purple";
            else if (detectedColor == 2) return "Green";
            else return "None";
        }
    }
}
