package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.vision.AsyncAprilTagLocalizer;

/**
 * TeleOp to test AprilTag-based localization (DECODE).
 * Displays robot pose from detected AprilTags in telemetry.
 */
@TeleOp(name = "Test AprilTag Localizer", group = "Test")
public class TestAprilTagLocalizer extends OpMode {

    private AsyncAprilTagLocalizer tagLocalizer;

    @Override
    public void init() {
        telemetry.addLine("Starting AprilTag Localizer...");
        telemetry.update();

        // Example camera offset: if camera is at robot origin, set all to zero.
        // (x,y in meters, heading in radians)
        tagLocalizer = new AsyncAprilTagLocalizer(hardwareMap,
                0.0, // cameraOffsetX_m
                0.0, // cameraOffsetY_m
                0.0  // cameraOffsetHeading_rad
        );

        tagLocalizer.start();

        telemetry.addLine("AprilTag Localizer started.");
        telemetry.update();
    }

    @Override
    public void loop() {
        Pose pose = tagLocalizer.getPose();

        if (pose != null) {
            telemetry.addData("Robot Pose", "");
            telemetry.addData("X (m)", "%.2f", pose.getX());
            telemetry.addData("Y (m)", "%.2f", pose.getY());
            telemetry.addData("Heading (rad)", "%.2f", pose.getHeading());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.getHeading()));
        } else {
            telemetry.addLine("No AprilTag detected.");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        tagLocalizer.stop();
    }
}
