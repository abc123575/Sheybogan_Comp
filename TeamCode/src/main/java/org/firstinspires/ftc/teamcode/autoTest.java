package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Pose2d;

// No visualizer because Quickstart Pedro does not support it

@Autonomous(name="autoTest", group="Auto")
public class autoTest extends LinearOpMode {

    private DcMotor intakeMotor, LLaunch, RLaunch;
    private Follower follower;

    private static final double LAUNCH_POWER = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        LLaunch     = hardwareMap.get(DcMotor.class, "LLaunch");
        RLaunch     = hardwareMap.get(DcMotor.class, "RLaunch");

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ---------------------------------------------------------
        // A: TURN UNTIL CAMERA SEES TAG (SIMULATED)
        // ---------------------------------------------------------

        telemetry.addLine("Turning to find AprilTag...");
        telemetry.update();

        // spin slowly
        while (opModeIsActive() && !cameraDetectsTag()) {
            follower.setDrivePowers(0, 0, 0.3);
            follower.update();
        }

        follower.setDrivePowers(0, 0, 0);
        sleep(200);

        // ---------------------------------------------------------
        // B: TURN LEFT 45°
        // ---------------------------------------------------------

        turnDegrees(45);

        // ---------------------------------------------------------
        // C: Reset odometry
        // ---------------------------------------------------------
        follower.setPose(new Pose2d(0, 0, 0));

        // ---------------------------------------------------------
        // D: Turn left another 45° (total 90)
        // ---------------------------------------------------------
        turnDegrees(45);

        // ---------------------------------------------------------
        // E: Drive forward 60 inches
        // ---------------------------------------------------------
        driveForwardInches(60, 0.5);

        // ---------------------------------------------------------
        // F: Turn right 135°
        // ---------------------------------------------------------
        turnDegrees(-135);

        // ---------------------------------------------------------
        // G: Drive forward 396 inches (33 ft)
        // ---------------------------------------------------------
        driveForwardInches(396, 0.8);

        // ---------------------------------------------------------
        // H: Run intake -2
        // ---------------------------------------------------------
        intakeMotor.setPower(-1);   // old SDK does not allow >1 power

        // ---------------------------------------------------------
        // I: Drive forward 5 seconds
        // ---------------------------------------------------------
        follower.setDrivePowers(0.5, 0, 0);
        sleep(5000);
        follower.setDrivePowers(0,0,0);

        // ---------------------------------------------------------
        // J: Turn again until find April tag
        // ---------------------------------------------------------
        while (opModeIsActive() && !cameraDetectsTag()) {
            follower.setDrivePowers(0,0,0.2);
            follower.update();
        }
        follower.setDrivePowers(0,0,0);

        // ---------------------------------------------------------
        // K: Align with tag (dummy estimate)
        // ---------------------------------------------------------
        Pose2d tagPose = new Pose2d(10, 10, 0);
        goTo(tagPose);

        // ---------------------------------------------------------
        // L: Shoot
        // ---------------------------------------------------------
        LLaunch.setPower(LAUNCH_POWER);
        RLaunch.setPower(LAUNCH_POWER);
        sleep(1200);
        LLaunch.setPower(0);
        RLaunch.setPower(0);

        telemetry.addLine("Auto Complete!");
        telemetry.update();
    }


    // ============================================================
    // Helper functions for QUICKSTART Pedro Pathing
    // ============================================================

    private void driveForwardInches(double inches, double speed) {
        double seconds = inches / 20.0; // simple estimate
        follower.setDrive
