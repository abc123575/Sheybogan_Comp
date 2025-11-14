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

@TeleOp(name = "TELEOP_Final", group = "Drive")
public class TELEOP_Final extends OpMode {

    // --- Drive Motors ---
    private DcMotor FLmotor, FRmotor, BLmotor, BRmotor;

    // --- Launcher Motors ---
    private DcMotor LLaunch, RLaunch;

    // --- Intake + Kickstand Motors ---
    private DcMotor intakeMotor, kickstandMotor;

    // --- IMU ---
    private IMU imu;

    // --- Field Centric Variables ---
    private boolean fieldCentric = false;
    private boolean backPrev = false;
    private boolean yPrev = false;

    private static final double DEADBAND = 0.05;
    private static final double ROT_SCALE = 0.8;

    // --- Power Constants ---
    private static final double LAUNCH_POWER = 1 ;
    private static final double INTAKE_POWER = 1.0;
    private static final double KICKSTAND_POWER = 0.8;

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

        // --- Intake + Kickstand ---
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        kickstandMotor = hardwareMap.get(DcMotor.class, "kickstand_motor");

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        kickstandMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{intakeMotor, kickstandMotor}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // --- IMU Setup ---
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(logo, usb));
        imu.initialize(imuParams);

        telemetry.addLine("✅ TELEOP_FINAL Initialized");
        telemetry.addLine("Gamepad1 = Drive | Gamepad2 = Launcher + Intake + Kickstand");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Gamepad1: Drive Controls ---
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x * ROT_SCALE;

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

        // --- Field Centric Transform ---
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

        // --- Mecanum Drive Mix ---
        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        // Normalize
        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max; fr /= max; bl /= max; br /= max;

        // Apply power
        FLmotor.setPower(fl);
        FRmotor.setPower(fr);
        BLmotor.setPower(bl);
        BRmotor.setPower(br);

        // --- Gamepad2: Launcher (Right Trigger) ---
        if (gamepad2.right_trigger > 0.1) {
            LLaunch.setPower(LAUNCH_POWER);
            RLaunch.setPower(LAUNCH_POWER);
        } else if (gamepad2.left_trigger > 0.1) {
            LLaunch.setPower(-LAUNCH_POWER); // reverse if you want opposite spin
            RLaunch.setPower(-LAUNCH_POWER);
        } else {
            LLaunch.setPower(0);
            RLaunch.setPower(0);
        }


        // --- Gamepad2: Intake (Right Bumper) ---
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        if (gamepad2.right_bumper) {
            intakeMotor.setPower((INTAKE_POWER) * 2) ;
        } else {
            intakeMotor.setPower(0);
        }

        // --- Gamepad2: Kickstand (Left Bumper) ---
        if (gamepad2.left_bumper) {
            kickstandMotor.setPower(KICKSTAND_POWER);
        } else {
            kickstandMotor.setPower(0);
        }

        // --- Telemetry ---
        telemetry.addLine("Drive:");
        telemetry.addData("Field Centric", fieldCentric);
        telemetry.addData("Heading (deg)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        telemetry.addLine("\nLauncher:");
        telemetry.addData("Right Trigger", gamepad2.right_trigger);
        telemetry.addData("LLaunch Power", LLaunch.getPower());
        telemetry.addData("RLaunch Power", RLaunch.getPower());

        telemetry.addLine("\nIntake:");
        telemetry.addData("Right Bumper", gamepad2.right_bumper);
        telemetry.addData("Intake Power", intakeMotor.getPower());

        telemetry.addLine("\nKickstand:");
        telemetry.addData("Left Bumper", gamepad2.left_bumper);
        telemetry.addData("Kickstand Power", kickstandMotor.getPower());

        telemetry.update();
    }
}
