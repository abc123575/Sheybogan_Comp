package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.ftc.localization.constants.PinpointConstants;

import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "AutoFinal", group = "Pedro")
public class MyAutoV2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //----------------------------------------------------------------------
        // MECANUM DRIVE SETUP
        //----------------------------------------------------------------------
        MecanumConstants drive = new MecanumConstants();
        drive.leftFrontMotorName = "FLmotor";
        drive.leftRearMotorName = "BLmotor";
        drive.rightFrontMotorName = "FRmotor";
        drive.rightRearMotorName = "BRmotor";

        drive.leftFrontMotorDirection = com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
        drive.leftRearMotorDirection = com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
        drive.rightFrontMotorDirection = com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
        drive.rightRearMotorDirection = com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

        Mecanum mecanum = new Mecanum(hardwareMap, drive);

        //----------------------------------------------------------------------
        // PINPOINT ODOMETRY (YOUR TUNED VALUES APPLIED)
        //----------------------------------------------------------------------
        PinpointConstants odo = new PinpointConstants()
                .forwardPodY(4)
                .strafePodX(5)
                .distanceUnit(DistanceUnit.INCH)
                .hardwareMapName("pinpoint")
                .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
                .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Create placeholder starting pose (heading updated before start)
        Pose startPose = new Pose(56, 8, 0);

        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, odo, startPose);

        //----------------------------------------------------------------------
        // FOLLOWER SETUP (YOUR TUNE INCLUDED)
        //----------------------------------------------------------------------
        FollowerConstants constants = new FollowerConstants()
                .forwardZeroPowerAcceleration(-29.542932374300786)
                .lateralZeroPowerAcceleration(-41.32537990883812)
                .mass(9);

        Follower follower = new Follower(constants, localizer, mecanum);

        //----------------------------------------------------------------------
        // AUTO-DETECT STARTING HEADING BEFORE MATCH
        //----------------------------------------------------------------------
        localizer.update();
        double detectedHeading = localizer.getPose().getHeading(); // radians

        telemetry.addData("Detected Heading (deg)", Math.toDegrees(detectedHeading));
        telemetry.update();
        sleep(500); // OPTIONAL: show heading to drivers

        // Apply the real detected heading
        startPose = new Pose(56, 8, detectedHeading);
        follower.setStartingPose(startPose);

        //----------------------------------------------------------------------
        // LOAD PATHS
        //----------------------------------------------------------------------
        Paths paths = new Paths(follower);

        //----------------------------------------------------------------------
        // WAIT FOR START
        //----------------------------------------------------------------------
        waitForStart();

        //----------------------------------------------------------------------
        // RUN PATH 1
        //----------------------------------------------------------------------
        follower.followPath(paths.Path1);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }

        //----------------------------------------------------------------------
        // RUN PATH 2
        //----------------------------------------------------------------------
        follower.followPath(paths.Path2);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
    }

    // ======================================================================
    // PATH DEFINITIONS
    // ======================================================================

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 8.000), new Pose(56.000, 36.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 36.000), new Pose(114.182, 36.121))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();
        }
    }

}
