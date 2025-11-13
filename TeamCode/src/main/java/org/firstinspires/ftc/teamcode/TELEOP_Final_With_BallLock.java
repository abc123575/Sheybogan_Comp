package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.bylazar.camerastream.PanelsCameraStream;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.opencv.imgproc.Moments;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;

import org.opencv.android.Utils;
import org.opencv.core.*;

import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;

@TeleOp(name="TELEOP_Final_With_BallLock", group="Drive")
public class TELEOP_Final_With_BallLock extends OpMode {

    private DcMotor FLmotor, FRmotor, BLmotor, BRmotor;
    private DcMotor LLaunch, RLaunch;
    private DcMotor intakeMotor, kickstandMotor;

    private IMU imu;

    private boolean fieldCentric = false;
    private boolean backPrev = false;
    private boolean yPrev = false;

    private static final double ROT_SCALE = 0.8;
    private static final double DEADBAND = 0.05;

    private VisionPortal portal;
    private StreamProcessor streamProcessor;

    private boolean autoTurn = false;
    private boolean aPrev = false;

    @Override
    public void init() {

        FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
        FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");
        BLmotor = hardwareMap.get(DcMotor.class, "BLmotor");
        BRmotor = hardwareMap.get(DcMotor.class, "BRmotor");

        FLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BLmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{FLmotor, FRmotor, BLmotor, BRmotor}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        LLaunch = hardwareMap.get(DcMotor.class, "LLaunch");
        RLaunch = hardwareMap.get(DcMotor.class, "RLaunch");
        RLaunch.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        kickstandMotor = hardwareMap.get(DcMotor.class, "kickstand_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        kickstandMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        streamProcessor = new StreamProcessor();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(streamProcessor)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        PanelsCameraStream.INSTANCE.startStream(streamProcessor, 15);

        telemetry.addLine("✅ Initialized + Panels Stream Active");
        telemetry.update();
    }

    @Override
    public void loop() {

        boolean aNow = gamepad1.a;
        if (aNow && !aPrev) autoTurn = !autoTurn;
        aPrev = aNow;

        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x * ROT_SCALE;

        if (Math.abs(lx) < DEADBAND) lx = 0;
        if (Math.abs(ly) < DEADBAND) ly = 0;
        if (Math.abs(rx) < DEADBAND) rx = 0;

        if (gamepad1.back && !backPrev)
            fieldCentric = !fieldCentric;
        if (gamepad1.y && !yPrev)
            imu.resetYaw();

        backPrev = gamepad1.back;
        yPrev = gamepad1.y;

        double x = lx;
        double y = ly;

        if (fieldCentric) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double cosA = Math.cos(-heading);
            double sinA = Math.sin(-heading);
            double rotX = x * cosA - y * sinA;
            double rotY = x * sinA + y * cosA;
            x = rotX;
            y = rotY;
        }

        // ===== AUTO TURN WITH PROPORTIONAL CONTROL =====
        if (autoTurn) {
            double turnPower = streamProcessor.getTurnPower();

            rx = -turnPower;  // invert because camera upside down
            x = 0;            // disable strafe while aiming
        }

        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        FLmotor.setPower(fl / max);
        FRmotor.setPower(fr / max);
        BLmotor.setPower(bl / max);
        BRmotor.setPower(br / max);

        if (gamepad2.right_trigger > 0.1) {
            LLaunch.setPower(1);
            RLaunch.setPower(1);
        } else if (gamepad2.left_trigger > 0.1) {
            LLaunch.setPower(-1);
            RLaunch.setPower(-1);
        } else {
            LLaunch.setPower(0);
            RLaunch.setPower(0);
        }

        intakeMotor.setPower((gamepad2.right_bumper ? 1.0 : 0)*2);
        kickstandMotor.setPower(gamepad2.left_bumper ? 0.8 : 0);

        telemetry.addData("AutoTurn (A)", autoTurn);
        telemetry.addData("Detected Color", streamProcessor.getDetectedColorName());
        telemetry.addData("Turn Power", streamProcessor.getTurnPower());
        telemetry.addData("Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
    }

    // ===================================================================
    //                STREAM PROCESSOR
    // ===================================================================
    class StreamProcessor implements VisionProcessor, CameraStreamSource {

        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        private int detectedColor = 0;
        private int ballX = -1;

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long timestamp) {

            Mat hsv = new Mat();
            Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

            Scalar lowerPurple = new Scalar(115, 50, 50);
            Scalar upperPurple = new Scalar(165, 255, 255);
            Scalar lowerGreen = new Scalar(30, 40, 40);
            Scalar upperGreen = new Scalar(90, 255, 255);

            Mat purpleMask = new Mat();
            Mat greenMask = new Mat();

            Core.inRange(hsv, lowerPurple, upperPurple, purpleMask);
            Core.inRange(hsv, lowerGreen, upperGreen, greenMask);

            int pc = Core.countNonZero(purpleMask);
            int gc = Core.countNonZero(greenMask);

            if (pc > 2000 && pc > gc) detectedColor = 1;
            else if (gc > 2000 && gc > pc) detectedColor = 2;
            else detectedColor = 0;

            Mat chosen = (detectedColor == 1) ? purpleMask : greenMask;

            if (detectedColor != 0) {
                Moments m = Imgproc.moments(chosen);
                if (m.m00 > 0) {
                    ballX = (int) (m.m10 / m.m00);
                }
            } else {
                ballX = -1;
            }

            Bitmap bmp = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, bmp);
            lastFrame.set(bmp);

            hsv.release();
            purpleMask.release();
            greenMask.release();

            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int w, int h, float x, float y, Object ctx) {}

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(c -> c.accept(lastFrame.get()));
        }

        public String getDetectedColorName() {
            return detectedColor == 1 ? "Purple" : detectedColor == 2 ? "Green" : "None";
        }

        // 🚀 NEW PROPORTIONAL TURN CONTROL
        public double getTurnPower() {

            if (ballX < 0) return 0;  // no ball

            int center = lastFrame.get().getWidth() / 2;
            int error = ballX - center;

            double normalized = (double) error / center;

            normalized = Math.max(-1, Math.min(1, normalized));

            double kP = 0.80;
            double power = normalized * kP;

            if (Math.abs(power) < 0.05) power = 0;

            return power;
        }
    }
}
