package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "autoTest", group = "Auto")
public class autoTest extends LinearOpMode {

    private Follower follower;

    // Define a few poses in field coordinates (inches, radians)
    private Pose startPose   = new Pose(0, 0, 0);                        // start
    private Pose forwardPose = new Pose(0, 60, 0);                       // 60" forward
    private Pose endPose     = new Pose(0, 60, Math.toRadians(-90));     // same spot, turned

    @Override
    public void runOpMode() throws InterruptedException {

        // Create Pedro follower from your Constants class
        follower = Constants.createFollower(hardwareMap);

        // Set initial pose for the follower
        follower.setPose(startPose);

        telemetry.addLine("autoTest ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ------------------------------------------------------------
        // Build a simple path chain:
        //  1) Move from startPose -> forwardPose
        //  2) While moving, linearly turn from 0° to +45°
        //  3) Then continue turning to -90° at the end
        // ------------------------------------------------------------
        PathChain pathChain = follower.pathBuilder()
                // drive from startPose to forwardPose
                .addPath(new BezierLine(startPose, forwardPose))
                // smoothly turn from 0 rad to +45° as we move
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        Math.toRadians(45),
                        0.5
                )
                // then finish turning to -90° near the end of the path
                .setLinearHeadingInterpolation(
                        Math.toRadians(45),
                        endPose.getHeading(),
                        1.0
                )
                .build();

        // Follow the built path
        follower.followPath(pathChain);

        // Keep updating follower while the path is running
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }

        // Stop at the end
        follower.breakFollowing();
    }
}
