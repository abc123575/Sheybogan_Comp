package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * TELEOP_FINAL
 * -------------
 * Gamepad 1: Mecanum Drive (Field-Centric)
 * Gamepad 2: Launcher + Intake Control
 */
@TeleOp(name = "TELEOP_Final", group = "Drive")
public class TELEOP_Final extends OpMode {

    // --- Drive Motors ---
    private DcMotor FLmotor, FRmotor, BLmotor, BRmotor;

    // --- Launcher Motors ---
    private DcMotor LLaunch, RLaunch;

    // --- Intake Motor ---
    private DcMotor intakeMotor;

    // --- IMU ---
    private IMU imu;

    // --- Field Centric Variables ---
    private boolean fieldCentric = false;
    private boolean backPrev = false;
    private boolean yPrev = false;

    private static final double DEADBAND = 0.05;
    private static final double ROT_SCALE = 0.8;

    // --- Launcher Control ---
    private static final double LAUNCH_POWER = 0.8;  // Adjust as needed
    private boolean launchOn = false;
    private boolean aPrev = false;

    // --- Intake Control ---
    private boolean intakeOn = false;
    private boolean rbPrev = false;

    @Override
    public void init() {
        // --- Drive Motors ---
        FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
        FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");
        BLmotor = hardwareMap.get(DcMotor.class, "BLmotor");
        BRmotor = hardwareMap.get(DcMotor.class, "BRmotor");

        FLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FRmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BRmotor.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{FLmotor, FRmotor, BLmotor, BRmotor}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // --- Launcher Motors ---
        LLaunch = hardwareMap.get(DcMotor.class, "LLaunch");
        RLaunch = hardwareMap.get(DcMotor.class, "RLaunch");

        RLaunch.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{LLaunch, RLaunch}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // --- Intake Motor ---
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- IMU ---
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(logo, usb));
        imu.initialize(imuParams);

        telemetry.addLine("✅ TELEOP_FINAL Initialized");
        telemetry.addLine("Gamepad1 = Drive | Gamepad2 = Launcher + Intake");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Gamepad1: Drive Controls ---
        double lx = gamepad1.left_stick_x;    // strafe
        double ly = -gamepad1.left_stick_y;   // forward/back
        double rx = gamepad1.right_stick_x * ROT_SCALE; // rotation

        // Deadband
        lx = (Math.abs(lx) < DEADBAND) ? 0 : lx;
        ly = (Math.abs(ly) < DEADBAND) ? 0 : ly;
        rx = (Math.abs(rx) < DEADBAND) ? 0 : rx;

        // --- Toggle field centric ---
        boolean backNow = gamepad1.back;
        if (backNow && !backPrev) fieldCentric = !fieldCentric;
        backPrev = backNow;

        // --- Reset heading ---
        boolean yNow = gamepad1.y;
        if (yNow && !yPrev) imu.resetYaw();
        yPrev = yNow;

        // --- Field Centric transform ---
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
        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        // Normalize
        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        // Apply power
        FLmotor.setPower(fl);
        FRmotor.setPower(fr);
        BLmotor.setPower(bl);
        BRmotor.setPower(br);

        // --- Gamepad2: Launcher ---
        boolean aNow = gamepad2.a;
        if (aNow && !aPrev) {
            launchOn = !launchOn; // toggle
        }
        aPrev = aNow;

        if (launchOn) {
            LLaunch.setPower(LAUNCH_POWER);
            RLaunch.setPower(LAUNCH_POWER);
        } else {
            LLaunch.setPower(0);
            RLaunch.setPower(0);
        }

        // --- Gamepad2: Intake ---
        boolean rbNow = gamepad2.right_bumper;
        if (rbNow && !rbPrev) {
            intakeOn = !intakeOn;
        }
        rbPrev = rbNow;

        if (intakeOn) {
            intakeMotor.setPower(1.0); // Full power intake
        } else {
            intakeMotor.setPower(0.0);
        }

        // --- Telemetry ---
        telemetry.addLine("Drive:");
        telemetry.addData("Field Centric", fieldCentric);
        telemetry.addData("Heading (deg)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        telemetry.addLine("\nLauncher:");
        telemetry.addData("Launcher Active", launchOn);
        telemetry.addData("LLaunch Pos", LLaunch.getCurrentPosition());
        telemetry.addData("RLaunch Pos", RLaunch.getCurrentPosition());

        telemetry.addLine("\nIntake:");
        telemetry.addData("Intake Active", intakeOn);

        telemetry.update();
    }
}