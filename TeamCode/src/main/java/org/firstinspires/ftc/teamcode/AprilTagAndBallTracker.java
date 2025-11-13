package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.bylazar.camerastream.PanelsCameraStream;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Autonomous(name="AprilTag + Ball Tracker (Panels Stream)", group="Vision")
public class AprilTagAndBallTracker extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private StreamProcessor streamProcessor;

    @Override
    public void runOpMode() {

        // AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        // Combined processor with bitmap stream
        streamProcessor = new StreamProcessor();

        // Build VisionPortal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .addProcessor(streamProcessor)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        // ⭐ Start Panels stream (CORRECT API)
        PanelsCameraStream.INSTANCE.startStream(streamProcessor, 15);

        telemetry.addLine("Ready. Panels streaming enabled.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            int id = detections.isEmpty() ? -1 : detections.get(0).id;

            telemetry.addData("AprilTag ID", id);
            telemetry.addData("Ball Color", streamProcessor.getDetectedColorName());
            telemetry.update();
        }

        // ⭐ Stop stream (CORRECT API)
        PanelsCameraStream.INSTANCE.stopStream();
        visionPortal.close();
    }

    // -------------------------------------------------------------------------
    // STREAM PROCESSOR
    // -------------------------------------------------------------------------

    class StreamProcessor implements VisionProcessor, CameraStreamSource {

        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1,1, Bitmap.Config.RGB_565));

        private int detectedColor = 0;
        private double purpleCount = 0;
        private double greenCount = 0;

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long timestampNs) {

            // --- HSV COLOR DETECTION ---
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

            purpleCount = Core.countNonZero(purpleMask);
            greenCount = Core.countNonZero(greenMask);

            if (purpleCount > 2000 && purpleCount > greenCount)
                detectedColor = 1;
            else if (greenCount > 2000 && greenCount > purpleCount)
                detectedColor = 2;
            else
                detectedColor = 0;

            hsv.release();
            purpleMask.release();
            greenMask.release();

            // Convert frame to Bitmap
            Bitmap bmp = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, bmp);
            lastFrame.set(bmp);

            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int w, int h, float x, float y, Object ctx) {}

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(c -> c.accept(lastFrame.get()));
        }

        public String getDetectedColorName() {
            if (detectedColor == 1) return "Purple";
            if (detectedColor == 2) return "Green";
            return "None";
        }
        //hi
    }
}
//