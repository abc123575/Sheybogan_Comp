package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.ftc.localization.constants.PinpointConstants;

import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierCurve;

@Autonomous(name = "AutoFinal", group = "Pedro")
public class autoFinalV1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // --------------------------------------------------------------
        // CONFIGURE MECANUM DRIVE CONSTANTS
        // --------------------------------------------------------------
        MecanumConstants drive = new MecanumConstants();

        drive.leftFrontMotorName  = "FLmotor";
        drive.leftRearMotorName   = "BLmotor";
        drive.rightFrontMotorName = "FRmotor";
        drive.rightRearMotorName  = "BRmotor";

        // Adjust directions if robot moves incorrectly
        drive.leftFrontMotorDirection  = com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
        drive.leftRearMotorDirection   = com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
        drive.rightFrontMotorDirection = com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
        drive.rightRearMotorDirection  = com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

        // Initialize drivetrain
        Mecanum mecanum = new Mecanum(hardwareMap, drive);

        // --------------------------------------------------------------
        // CONFIGURE PINPOINT LOCALIZER
        // --------------------------------------------------------------
        PinpointConstants odo = new PinpointConstants();
        odo.hardwareMapName = "pipoint";     // your config name
        odo.forwardPodY = 0;                 // adjust if needed
        odo.strafePodX  = 0;                 // adjust if needed

        // Starting pose from Visualizer (.pp)
        Pose startPose = new Pose(56, 8, Math.toRadians(90));

        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, odo, startPose);

        // --------------------------------------------------------------
        // FOLLOWER SETUP
        // --------------------------------------------------------------
        FollowerConstants constants = new FollowerConstants();
        Follower follower = new Follower(constants, localizer, mecanum);

        // Tell follower the starting pose
        follower.setStartingPose(startPose);

        // --------------------------------------------------------------
        // PATH FROM YOUR (.pp) FILE - CONVERTED TO BEZIER CURVES
        // --------------------------------------------------------------
        PathChain chain = follower.pathBuilder()

                // Path 1
                .addPath(new BezierCurve(
                        new Pose(56, 8, Math.toRadians(90)),
                        new Pose(83.0930, 36, Math.toRadians(180))
                ))

                // Path 2
                .addPath(new BezierCurve(
                        new Pose(83.0930, 36, Math.toRadians(180)),
                        new Pose(43.9534, 67.6046, Math.toRadians(180))
                ))

                .build();

        // --------------------------------------------------------------
        // WAIT + RUN
        // --------------------------------------------------------------
        waitForStart();

        follower.followPath(chain);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
    }
}
