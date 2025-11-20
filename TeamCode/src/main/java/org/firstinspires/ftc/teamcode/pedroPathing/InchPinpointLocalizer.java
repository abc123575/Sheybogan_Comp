package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This wrapper converts PinpointLocalizer's METER output into INCHES.
 * Pedro Pathing expects inches, Visualizer exports inches, so this fixes scaling.
 */
public class InchPinpointLocalizer extends PinpointLocalizer {

    private static final double METERS_TO_INCHES = 39.37007874; // exact conversion

    public InchPinpointLocalizer(HardwareMap hardwareMap, PinpointConstants constants, Pose startPose) {
        super(hardwareMap, constants, startPose);
    }

    @Override
    public Pose getPose() {
        Pose p = super.getPose();

        // convert meters → inches
        return new Pose(
                p.getX() * METERS_TO_INCHES,
                p.getY() * METERS_TO_INCHES,
                p.getHeading()     // heading stays in radians
        );
    }
}
