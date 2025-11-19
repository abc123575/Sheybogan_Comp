package org.firstinspires.ftc.teamcode.pedroPathing;

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
        drive.leftFrontMotorName  = "FLmotor";
        drive.leftRearMotorName   = "BLmotor";
        drive.rightFrontMotorName = "FRmotor";
        drive.rightRearMotorName  = "BRmotor";

        drive.leftFrontMotorDirection  = com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
        drive.leftRearMotorDirection   = com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
        drive.rightFrontMotorDirection = com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
        drive.rightRearMotorDirection  = com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

        Mecanum mecanum = new Mecanum(hardwareMap, drive);

        //----------------------------------------------------------------------
        // PINPOINT ODOMETRY (YOUR TUNED VALUES APPLIED)
        //----------------------------------------------------------------------
        PinpointConstants odo = new PinpointConstants()
                .forwardPodY(9)     // inches — correct
                .strafePodX(7)      // inches — correct
                .distanceUnit(DistanceUnit.INCH)
                .hardwareMapName("pinpoint")
                .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Starting pose from your Visualizer
        Pose startPose = new Pose(56, 8, Math.toRadians(90));

        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, odo, startPose);

        //----------------------------------------------------------------------
        // FOLLOWER SETUP (YOUR TUNE INCLUDED)
        //----------------------------------------------------------------------
        FollowerConstants constants = new FollowerConstants()
                .forwardZeroPowerAcceleration(-29.542932374300786)
                .lateralZeroPowerAcceleration(-41.32537990883812)
                .mass(9);

        Follower follower = new Follower(constants, localizer, mecanum);
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

            // ------------------ Path 1 ------------------
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new com.pedropathing.geometry.BezierLine(
                                    new Pose(56.000, 8.000),
                                    new Pose(83.093, 36.000)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),     // start heading
                            Math.toRadians(180)     // smooth rotation
                    )
                    .build();

            // ------------------ Path 2 ------------------
            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new com.pedropathing.geometry.BezierLine(
                                    new Pose(83.093, 36.000),
                                    new Pose(43.953, 67.605)
                            )
                    )
                    .setTangentHeadingInterpolation() // automatic curve-based heading
                    .build();
        }
    }
}
