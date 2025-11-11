package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.atomic.AtomicReference;

@TeleOp(name = "Mecanum + UDP Spin", group = "Drive")
public class Mecanum_Wheels_test extends OpMode {

    // Motors
    private DcMotor FLmotor, FRmotor, BLmotor, BRmotor, Llaunch;
    private IMU imu;

    // Field-centric and control toggles
    private boolean fieldCentric = false;
    private boolean backPrev = false;
    private boolean yPrev = false;

    // Rotation control
    private static final double DEADBAND = 0.05;
    private static final double ROT_SCALE = 0.8;

    // --- UDP config ---
    private static final int UDP_PORT = 9000;
    private static final int UDP_TIMEOUT_MS = 200;
    private volatile boolean running = false;
    private Thread udpThread;
    private DatagramSocket udpSocket;
    private final AtomicReference<Double> udpSpin = new AtomicReference<>(0.0);

    @Override
    public void init() {
        // --- Hardware map ---
        FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
        FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");
        BLmotor = hardwareMap.get(DcMotor.class, "BLmotor");
        BRmotor = hardwareMap.get(DcMotor.class, "BRmotor");

        // --- Motor directions ---
        FLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FRmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BRmotor.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{FLmotor, FRmotor, BLmotor, BRmotor}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // --- IMU init ---
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(logo, usb));
        imu.initialize(imuParams);

        // --- Start UDP listener thread ---
        running = true;
        udpThread = new Thread(this::listenUdp, "UDP-Spin-Listener");
        udpThread.setDaemon(true);
        udpThread.start();


        telemetry.addLine("Mecanum + UDP Spin initialized");
        telemetry.addLine("LS = move | RS X = manual turn | UDP = camera spin correction");
        telemetry.update();
    }

    // --- Background thread: listen for UDP packets ---
    private void listenUdp() {
        byte[] buffer = new byte[64];
        DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
        try (DatagramSocket socket = new DatagramSocket(UDP_PORT)) {
            udpSocket = socket;
            socket.setSoTimeout(UDP_TIMEOUT_MS);
            RobotLog.d("UDP spin socket listening on port " + UDP_PORT);

            while (running && !Thread.currentThread().isInterrupted()) {
                try {
                    socket.receive(packet);
                    String data = new String(packet.getData(), 0, packet.getLength(), StandardCharsets.UTF_8).trim();
                    double value;
                    try {
                        value = Double.parseDouble(data);
                    } catch (NumberFormatException nfe) {
                        RobotLog.e("UDP parse error: '" + data + "'");
                        continue;
                    }

                    value = Range.clip(value, -1.0, 1.0); // normalize between -1 and 1
                    udpSpin.set(value);

                } catch (SocketTimeoutException ste) {
                    // ignore timeouts
                } catch (Exception e) {
                    if (running) RobotLog.e("UDP receive error: " + e.getMessage());
                }
            }
        } catch (Exception e) {
            RobotLog.e("UDP socket error: " + e.getMessage());
        } finally {
            udpSocket = null;
            RobotLog.d("UDP spin listener stopped.");
        }
    }

    @Override
    public void loop() {
        // --- Gamepad input ---
        double lx = gamepad1.left_stick_x;   // strafe
        double ly = -gamepad1.left_stick_y;  // forward/back
        double rx = gamepad1.right_stick_x * ROT_SCALE; // manual turn

        // --- UDP rotation value ---
        double udpVal = udpSpin.get(); // from camera (- = left, + = right)
        double udpTurn = Range.clip(udpVal * 0.6, -1, 1); // scale to limit spin speed

        // Deadband
        lx = (Math.abs(lx) < DEADBAND) ? 0 : lx;
        ly = (Math.abs(ly) < DEADBAND) ? 0 : ly;
        rx = (Math.abs(rx) < DEADBAND) ? 0 : rx;

        // Combine manual rotation and camera rotation
        double totalTurn = rx + udpTurn;
        totalTurn = Range.clip(totalTurn, -1.0, 1.0);

        // --- Field-centric toggle ---
        boolean backNow = gamepad1.back;
        if (backNow && !backPrev) fieldCentric = !fieldCentric;
        backPrev = backNow;

        // --- Reset heading ---
        boolean yNow = gamepad1.y;
        if (yNow && !yPrev) imu.resetYaw();
        yPrev = yNow;

        // --- Field-centric transform ---
        double x = lx;
        double y = ly;
        if (fieldCentric) {
            YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
            double heading = ypr.getYaw(AngleUnit.RADIANS);
            double cosA = Math.cos(-heading);
            double sinA = Math.sin(-heading);
            double rotX = x * cosA - y * sinA;
            double rotY = x * sinA + y * cosA;
            x = rotX;
            y = rotY;
        }

        // --- Mecanum drive mix ---
        double fl = y + x + totalTurn;
        double fr = y - x - totalTurn;
        double bl = y - x + totalTurn;
        double br = y + x - totalTurn;

        // Normalize
        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        // --- Apply power ---
        FLmotor.setPower(fl);
        FRmotor.setPower(fr);
        BLmotor.setPower(bl);
        BRmotor.setPower(br);

        // --- Telemetry ---
        telemetry.addData("UDP Spin", udpVal);
        telemetry.addData("Turn (combined)", totalTurn);
        telemetry.addData("Heading (deg)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Power", "FL %.2f | FR %.2f | BL %.2f | BR %.2f", fl, fr, bl, br);
        telemetry.update();
    }

    @Override
    public void stop() {
        running = false;
        if (udpSocket != null && !udpSocket.isClosed()) udpSocket.close();
        if (udpThread != null) udpThread.interrupt();
        for (DcMotor m : new DcMotor[]{FLmotor, FRmotor, BLmotor, BRmotor}) m.setPower(0);
    }
}
