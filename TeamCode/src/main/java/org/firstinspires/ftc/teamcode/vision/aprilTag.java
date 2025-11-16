package org.firstinspires.ftc.teamcode.vision;


import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Async AprilTag localizer for FTC DECODE season.
 * Uses FTC VisionPortal + AprilTagProcessor and provides
 * PedroPathing-compatible Pose.
 */
public class aprilTag implements Runnable {
    private final HardwareMap hardwareMap;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private Thread workerThread;
    private volatile boolean running = false;

    private final AtomicReference<Pose> latestPose = new AtomicReference<>(null);

    /**
     * ✅ Pass hardwareMap in constructor and assign it
     */
    public aprilTag(HardwareMap hw) {
        this.hardwareMap = hw;
    }

    /**
     * Start camera + processing in a background thread
     */
    public void start() {
        if (running) return;
        running = true;

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        workerThread = new Thread(this);
        workerThread.start();
    }

    /**
     * Stop background thread and close camera
     */
    public void stop() {
        running = false;
        if (workerThread != null) {
            try {
                workerThread.join();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    @Override
    public void run() {
        while (running) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (!detections.isEmpty()) {
                AprilTagDetection det = detections.get(0); // pick first for now

                // TODO: convert detection to field Pose using DECODE tag map
                Pose pose = convertDetectionToPose(det);
                latestPose.set(pose);
            }

            try {
                Thread.sleep(50); // avoid hogging CPU
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    /**
     * Get latest robot pose (or null if none)
     */
    public Pose getPose() {
        return latestPose.get();
    }

    private Pose convertDetectionToPose(AprilTagDetection det) {
        // 🔧 This is where you map tag ID -> known field pose (DECODE field tags)
        // and apply transform math from camera-to-tag.
        // For now just return a simple pose using FTC’s built-in estimate:
        return new Pose(det.ftcPose.x, det.ftcPose.y, Math.toRadians(det.ftcPose.yaw));
    }
}
