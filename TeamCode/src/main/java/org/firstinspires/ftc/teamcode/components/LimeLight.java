package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.bylazar.telemetry.TelemetryManager;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Configurable
public class LimeLight {
    private HardwareMap hardwareMap;
    public Limelight3A limelight;
    public LimeLight(HardwareMap hwm) {
        this.hardwareMap = hwm;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

    }

    public Pose3D update(ComponentShell Comps, TelemetryManager telemetryM, double robotYaw) {
        LLStatus status = limelight.getStatus();
        telemetryM.debug("Name", status.getName());
        telemetryM.debug("LL", "Temp: ", status.getTemp(), ", CPU: ", status.getCpu(), ", FPS: ", (int)status.getFps());
        telemetryM.debug("Pipeline", "Index: ", status.getPipelineIndex(), ", Type: ", status.getPipelineType());

        Pose3D BotposeMt2 = null;

        limelight.updateRobotOrientation(robotYaw);
        LLResult result = limelight.getLatestResult();

        while (result != null && result.isValid()) {
            if (result.getStaleness() < 100) {
                BotposeMt2 = result.getBotpose_MT2();
                if (BotposeMt2 != null) {
                    double x = BotposeMt2.getPosition().x;
                    double y = BotposeMt2.getPosition().y;
                    telemetryM.debug("MT2 Location:", "(" + x + ", " + y + ")");
                }
            }
        }
        return BotposeMt2;
    }
}