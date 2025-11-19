package org.firstinspires.ftc.teamcode.pedroPathing;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;   // REQUIRED
import org.opencv.core.Mat;   // REQUIRED for processFrame()

import java.util.List;

@TeleOp(name="AprilTag 3D Grid Demo", group="Vision")
public class AprilTag3DGridDemo extends OpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private Paint gridPaint, robotPaint, textPaint;

    @Override
    public void init() {

        // APRILTAG PROCESSOR
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .build();

        // VISION PORTAL + OVERLAY PROCESSOR
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .addProcessor(new OverlayProcessor())   // <-- FIXED
                .build();

        // DRAWING SETUP
        gridPaint = new Paint();
        gridPaint.setColor(Color.GREEN);
        gridPaint.setStrokeWidth(2);

        robotPaint = new Paint();
        robotPaint.setColor(Color.RED);
        robotPaint.setStrokeWidth(8);

        textPaint = new Paint();
        textPaint.setColor(Color.WHITE);
        textPaint.setTextSize(40);
    }

    @Override
    public void loop() {

        List<AprilTagDetection> detections = aprilTag.getDetections();

        double robotX = 0, robotY = 0, heading = 0;

        if (!detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);
            robotX = tag.ftcPose.x;
            robotY = tag.ftcPose.y;
            heading = tag.ftcPose.yaw;
        }

        telemetry.addData("X (in)", robotX);
        telemetry.addData("Y (in)", robotY);
        telemetry.addData("Heading (deg)", heading);
        telemetry.update();
    }


    // ======================================================================================
    //      OVERLAY PROCESSOR (EXACT METHOD SIGNATURES YOUR SDK EXPECTS)
    // ======================================================================================
    private class OverlayProcessor implements VisionProcessor {

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            // No initialization needed
        }

        @Override
        public Object processFrame(Mat frame, long timestamp) {
            // We do not modify frame data
            return null;
        }

        @Override
        public void onDrawFrame(
                Canvas canvas,
                int w,
                int h,
                float x,
                float y,
                Object ctx
        ) {
            drawOverlay(canvas);
        }
    }


    // ======================================================================================
    //      ORIGINAL OVERLAY DRAWING CODE (UNCHANGED)
    // ======================================================================================
    private void drawOverlay(Canvas canvas) {

        List<AprilTagDetection> detections = aprilTag.getDetections();

        double rx = 0, ry = 0, heading = 0;

        if (!detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);
            rx = tag.ftcPose.x;
            ry = tag.ftcPose.y;
            heading = tag.ftcPose.yaw;
        }

        int w = canvas.getWidth();
        int h = canvas.getHeight();

        int centerX = w / 2;
        int centerY = h - 200;
        int gridSpacing = 50;

        // Draw vertical grid lines
        for (int i = -10; i <= 10; i++) {
            int x = centerX + i * gridSpacing;
            canvas.drawLine(x, centerY - 600, x, centerY + 200, gridPaint);
        }

        // Draw horizontal grid lines
        for (int j = -10; j <= 10; j++) {
            int y = centerY + j * gridSpacing;
            canvas.drawLine(centerX - 600, y, centerX + 600, y, gridPaint);
        }

        // Robot dot
        float dotX = centerX + (float)(rx * 2.0);
        float dotY = centerY - (float)(ry * 2.0);

        canvas.drawCircle(dotX, dotY, 20, robotPaint);

        // Heading arrow
        float hx = (float)(dotX + 60 * Math.cos(Math.toRadians(heading)));
        float hy = (float)(dotY - 60 * Math.sin(Math.toRadians(heading)));
        canvas.drawLine(dotX, dotY, hx, hy, robotPaint);

        // Pose text
        canvas.drawText(
                "X=" + (int)rx + "  Y=" + (int)ry + "  H=" + (int)heading,
                50, 50,
                textPaint
        );
    }
}
