
package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.pedropathing.geometry.Pose; // adjust this import if your Pedro Pose class is elsewhere

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
/**
 * Async AprilTag localizer for DECODE.
 * Uses VisionPortal + AprilTagProcessor and returns pedro.Pathing Pose.
 *
 * Notes:
 *  - DECODE GOAL tags: red=24, blue=20 (manual). Use goal tags for nav; OBELISK tags 21-23 are not deterministic. :contentReference[oaicite:2]{index=2}
 *  - Concept sample shows detections provide robotPose when TagLibrary includes the season tags. :contentReference[oaicite:3]{index=3}
 */
public class AsyncAprilTagLocalizer implements Runnable {
    private final HardwareMap hardwareMap;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private Thread workerThread;
    private volatile boolean running = false;
    private final AtomicReference<Pose> latestPose = new AtomicReference<>(null);
    private final Object startStopLock = new Object();

    // If your project prefers meters, we'll convert inches->meters (1 in = 0.0254 m)
    private static final double INCH_TO_METER = 0.0254;

    // Optionally supply a small transform if you don't set cameraPose in the processor builder.
    // This is the offset from the camera lens center to your robot origin (robot coordinate frame).
    // Example: camera is 3 inches forward from robot center, 2 inches right, and 6 inches up.
    // If you set the cameraPose in AprilTagProcessor.Builder.setCameraPose(...), you can leave this at zero.
    private final double cameraOffsetX_m; // +x = right (meters)
    private final double cameraOffsetY_m; // +y = forward (meters)
    private final double cameraOffsetHeading_rad; // heading offset (radians), if camera rotated w.r.t robot forward

    public AsyncAprilTagLocalizer(HardwareMap hardwareMap,
                                  double cameraOffsetX_m,
                                  double cameraOffsetY_m,
                                  double cameraOffsetHeading_rad) {
        this.hardwareMap = hardwareMap;
        this.cameraOffsetX_m = cameraOffsetX_m;
        this.cameraOffsetY_m = cameraOffsetY_m;
        this.cameraOffsetHeading_rad = cameraOffsetHeading_rad;
    }

    /** start VisionPortal + AprilTag processor and thread */
    public void start() {
        synchronized (startStopLock) {
            if (running) return;
            running = true;

            // Build AprilTag processor. IMPORTANT: you should select the TagLibrary for DECODE,
            // or use the SDK's TagLibrary that contains this season's tags so detection.robotPose is computed.
            aprilTag = new AprilTagProcessor.Builder()
                    // you can set CameraPose here if you know exact camera-to-robot transform:
                    // .setCameraPose(new Position(DistanceUnit.INCH, xInch, yInch, zInch, 0),
                    //                new YawPitchRollAngles(AngleUnit.DEGREES, yaw, pitch, roll, 0))
                    // Optionally set lens intrinsics if you have camera calibration:
                    // .setLensIntrinsics(fx, fy, cx, cy)
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .build();

            VisionPortal.Builder builder = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    // you may enable live view in debug: .enableLiveView(true)
                    ;

            visionPortal = builder.build();

            workerThread = new Thread(this, "Apriltag-Localizer-Thread");
            workerThread.start();
        }
    }

    /** stop thread and close camera */
    public void stop() {
        synchronized (startStopLock) {
            running = false;
            if (workerThread != null) {
                try {
                    workerThread.join(250);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
            if (visionPortal != null) {
                visionPortal.close();
            }
        }
    }

    @Override
    public void run() {
        final ElapsedTime timer = new ElapsedTime();
        while (running) {
            try {
                if (aprilTag == null) {
                    Thread.sleep(50);
                    continue;
                }

                List<AprilTagDetection> detections = aprilTag.getDetections();

                if (detections != null && !detections.isEmpty()) {
                    // Prefer GOAL tags (20,24) for DECODE; skip obelisk tags (21-23) unless you intentionally use them.
                    AprilTagDetection best = null;
                    for (AprilTagDetection d : detections) {
                        if (d.metadata != null && d.metadata.name != null) {
                            String name = d.metadata.name.toLowerCase();
                            // SDK's sample default TagLibrary often labels tags with names (Goal, Obelisk, etc.)
                            if (name.contains("goal") || d.id == 20 || d.id == 24) {
                                best = d;
                                break;
                            }
                        }
                        // fallback to first detection
                        if (best == null) best = d;
                    }

                    if (best != null && best.robotPose != null) {
                        // best.robotPose is an SDK-provided "robot pose" (field-relative) when TagLibrary is used.
                        // robotPose.getPosition() fields are in the output units (default inches in sample).
                        // Convert to meters and adapt to Pedro Pose (x,y,heading).
                        double x_in = best.robotPose.getPosition().x; // X = right (inches) or as SDK defines
                        double y_in = best.robotPose.getPosition().y; // Y = forward (inches)
                        double yaw_deg = best.robotPose.getOrientation().getYaw(AngleUnit.DEGREES); // degrees

                        // convert inches -> meters
                        double x_m = x_in * INCH_TO_METER;
                        double y_m = y_in * INCH_TO_METER;
                        double heading_rad = Math.toRadians(yaw_deg);

                        // The SDK field coordinate axes: check sample: usually X=right, Y=forward from field origin.
                        // Pedro's Pose typically expects (x,y,heading) with x forward/right depending on your setup.
                        // Here we assume Pedro expects x = field x in meters, y = field y in meters, heading in radians.
                        // If necessary, swap/flip axes to match your Pedro coordinate convention.

                        // Apply camera->robot offset if you did not set cameraPose in the processor builder.
                        // The robot pose provided by the SDK is the robot origin pose if the SDK knows cameraPose.
                        // If you did NOT set cameraPose in builder, apply the small correction:
                        if (cameraOffsetX_m != 0.0 || cameraOffsetY_m != 0.0 || cameraOffsetHeading_rad != 0.0) {
                            // rotate offset by heading and subtract (since detection gives position of camera)
                            double cosH = Math.cos(heading_rad);
                            double sinH = Math.sin(heading_rad);
                            // offset in field coords = rotate(camera offset in robot coords by robot heading)
                            double offsetFieldX = cameraOffsetX_m * cosH - cameraOffsetY_m * sinH;
                            double offsetFieldY = cameraOffsetX_m * sinH + cameraOffsetY_m * cosH;
                            // robot position = camera position - offsetField (if robot origin is behind/left of camera)
                            x_m = x_m - offsetFieldX;
                            y_m = y_m - offsetFieldY;
                            heading_rad = heading_rad - cameraOffsetHeading_rad;
                        }

                        // Create Pedro Pose (adjust constructor to your Pedro Pose API)
                        Pose pedroPose = new Pose(x_m, y_m, heading_rad);
                        latestPose.set(pedroPose);
                    }
                }

                // throttle loop
                long sleepMs = 30;
                if (timer.milliseconds() < 30.0) {
                    Thread.sleep(sleepMs);
                }
                timer.reset();
            } catch (InterruptedException ie) {
                Thread.currentThread().interrupt();
            } catch (Exception ex) {
                // keep running but log if you have a logger (avoid throwing out of thread)
                ex.printStackTrace();
            }
        }
    }

    /** Returns last known pose (may be null if none) */
    public Pose getPose() {
        return latestPose.get();
    }
}
