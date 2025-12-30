package org.firstinspires.ftc.teamcode.components;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.bylazar.telemetry.TelemetryManager;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Configurable
public class LimeLight {
    private HardwareMap hardwareMap;
    public Limelight3A limelight;
    public LimeLight(HardwareMap hwm, ComponentShell.Alliance al) {
        this.hardwareMap = hwm;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        switch (al){
            case BLUE:
                limelight.pipelineSwitch(0);
            case RED:
                limelight.pipelineSwitch(0);
        }
        limelight.start();
    }

    public Pose update(ComponentShell Comps, TelemetryManager telemetryM, double robotYaw) {
        LLStatus status = limelight.getStatus();
        //telemetryM.debug("Name ", status.getName());
        //telemetryM.debug("LL ", "Temp: ", status.getTemp(), ", CPU: ", status.getCpu(), ", FPS: ", (int)status.getFps());
        telemetryM.debug("P Index: ", status.getPipelineIndex(), "P Type: ", status.getPipelineType());

        Pose3D BotposeMt2 = null;
        Pose pos = new Pose();
        limelight.updateRobotOrientation(robotYaw);
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            if (result.getStaleness() < 100) {
                BotposeMt2 = result.getBotpose();
                if (BotposeMt2 != null) {
                    pos = pos.withX(BotposeMt2.getPosition().x);
                    pos = pos.withY(BotposeMt2.getPosition().y);
                }
            }
        }
        return pos;
    }
}