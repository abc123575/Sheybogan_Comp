package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.pedropathing.visualization.PedroPathVisualizer;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="autoTest", group="Auto")
public class autoTest extends LinearOpMode {

    // Hardware
    private DcMotor intakeMotor, LLaunch, RLaunch;

    // Pedro follower
    private Follower follower;

    // AprilTag pipeline (placeholder)
    private AprilTagHelper aprilTagHelper;

    // constants
    private static final double LAUNCH_POWER = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Start Visualizer
        PedroPathVisualizer.start();

        // Build follower
        follower = Constants.createFollower(hardwareMap);

        // Motors
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        LLaunch     = hardwareMap.get(DcMotor.class, "LLaunch");
        RLaunch     = hardwareMap.get(DcMotor.class, "RLaunch");

        // Camera + tag helper
        aprilTagHelper = new AprilTagHelper(hardwareMap, telemetry);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ---------------------------------------------------------------------------------------------
        // 1. TURN UNTIL WE SEE APRIL TAG
        // ---------------------------------------------------------------------------------------------
        telemetry.addLine("Turning to find AprilTag...");
        telemetry.update();

        follower.setTeleOpHeading(0);

        while (opModeIsActive() && !aprilTagHelper.hasDetection()) {
            follower.setDrivePowers(0, 0, 0.3);
            follower.update();
        }
        follower.setDrivePowers(0, 0, 0);

        telemetry.addLine("AprilTag detected!");
        telemetry.update();
        sleep(300);


        // ---------------------------------------------------------------------------------------------
        // 2. TURN LEFT 45°
        // ---------------------------------------------------------------------------------------------
        follower.turnSync(Math.toRadians(45));


        // ---------------------------------------------------------------------------------------------
        // 3. ZERO ODOMETRY
        // ---------------------------------------------------------------------------------------------
        follower.setPose(new Pose2d(0, 0, 0));   // Reset X/Y position & heading

        sleep(300);


        // ---------------------------------------------------------------------------------------------
        // 4. TURN LEFT ANOTHER 45° (total 90° from start)
        // ---------------------------------------------------------------------------------------------
        follower.turnSync(Math.toRadians(45));


        // ---------------------------------------------------------------------------------------------
        // 5. DRIVE FORWARD 60 INCHES
        // ---------------------------------------------------------------------------------------------
        Path go60 = follower.pathBuilder()
                .lineTo(60, 0)
                .build();

        follower.followPath(go60);
        while (opModeIsActive() && !follower.isFinished()) follower.update();


        // ---------------------------------------------------------------------------------------------
        // 6. TURN RIGHT 135°
        // ---------------------------------------------------------------------------------------------
        follower.turnSync(Math.toRadians(-135));


        // ---------------------------------------------------------------------------------------------
        // 7. DRIVE 33 FEET (396 inches)
        // ---------------------------------------------------------------------------------------------
        Path go33ft = follower.pathBuilder()
                .lineTo(396, 0)
                .build();

        follower.followPath(go33ft);
        while (opModeIsActive() && !follower.isFinished()) follower.update();


        // ---------------------------------------------------------------------------------------------
        // 8. START INTAKE AT -2 POWER
        // ---------------------------------------------------------------------------------------------
        intakeMotor.setPower(-2);


        // ---------------------------------------------------------------------------------------------
        // 9. DRIVE FORWARD 5 SECONDS
        // ---------------------------------------------------------------------------------------------
        follower.setDrivePowers(0.5, 0, 0);

        sleep(5000);

        follower.setDrivePowers(0, 0, 0);


        // ---------------------------------------------------------------------------------------------
        // 10. FIND APRIL TAG AGAIN
        // ---------------------------------------------------------------------------------------------
        telemetry.addLine("Searching for AprilTag again...");
        telemetry.update();

        while (opModeIsActive() && !aprilTagHelper.hasDetection()) {
            follower.setDrivePowers(0, 0, 0.25);
            follower.update();
        }
        follower.setDrivePowers(0, 0, 0);

        Pose2d tagPose = aprilTagHelper.getPoseEstimate();


        // ---------------------------------------------------------------------------------------------
        // 11. GO TO APRIL TAG & ALIGN
        // ---------------------------------------------------------------------------------------------
        Path goToTag = follower.pathBuilder()
                .lineTo(tagPose.getX(), tagPose.getY())
                .turnTo(tagPose.getHeading())
                .build();

        follower.followPath(goToTag);
        while (opModeIsActive() && !follower.isFinished()) follower.update();


        // ---------------------------------------------------------------------------------------------
        // 12. SHOOT WITH CATAPULT
        // ---------------------------------------------------------------------------------------------
        LLaunch.setPower(LAUNCH_POWER);
        RLaunch.setPower(LAUNCH_POWER);

        sleep(1200);

        LLaunch.setPower(0);
        RLaunch.setPower(0);

        // ---------------------------------------------------------------------------------------------
        telemetry.addLine("autoTest COMPLETE!");
        telemetry.update();
        sleep(1000);
    }


    // -----------------------------------------------------------------------
    // SIMPLE APRILTAG HELPER CLASS
    // (You can replace this with your real pipeline later)
    // -----------------------------------------------------------------------
    private static class AprilTagHelper {

        public AprilTagHelper(HardwareMap hw, Telemetry tele) { }

        public boolean hasDetection() {
            return true; // pretend we always detect (replace later)
        }

        public Pose2d getPoseEstimate() {
            return new Pose2d(10, 10, 0); // dummy tag coordinates (replace)
        }
    }
}
