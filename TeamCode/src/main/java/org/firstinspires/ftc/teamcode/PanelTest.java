package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.*;
import com.pedropathing.localization.*;


/**
 * Simple telemetry test (no FTC Dashboard)
 * Displays variable values on the Driver Station panel
 */
@TeleOp(name = "Panel Test", group = "Test")
public class PanelTest extends OpMode {

    // Variables for testing
    private double testNumber = 0.0;
    private boolean testSwitch = false;
    private String testText = "Hello FTC!";

    @Override
    public void init() {
        telemetry.addLine("✅ Panel Test Initialized");
        telemetry.addLine("Use gamepad buttons to interact.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Example: control test values using buttons
        if (gamepad1.a) testNumber += 0.1;
        if (gamepad1.b) testNumber -= 0.1;
        if (gamepad1.x) testSwitch = !testSwitch;
        if (gamepad1.y) testText = "Button Y pressed!";

        // Display test variables on the Driver Station
        telemetry.addData("Test Number", "%.2f", testNumber);
        telemetry.addData("Test Switch", testSwitch);
        telemetry.addData("Test Text", testText);
        telemetry.update();
    }
}
