package org.firstinspires.ftc.teamcode.components;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.bylazar.telemetry.TelemetryManager;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Configurable
public class LimeLight {
	public Limelight3A limeLight;
    public LimeLight(HardwareMap hwm, ComponentShell.Alliance al) {
		limeLight = hwm.get(Limelight3A.class, "limelight");
        switch (al){
            case BLUE:
                limeLight.pipelineSwitch(0);
            case RED:
                limeLight.pipelineSwitch(0);
        }
        limeLight.start();
    }

    public Pose update(TelemetryManager telemetryM, double robotYaw) {
        LLStatus status = limeLight.getStatus();
        telemetryM.debug("P Index: ", status.getPipelineIndex(), "P Type: ", status.getPipelineType());

        Pose3D botPoseMt2;
        Pose pos = new Pose();
        limeLight.updateRobotOrientation(robotYaw);
        LLResult result = limeLight.getLatestResult();

        if (result != null && result.isValid()) {
            if (result.getStaleness() < 100) {
                botPoseMt2 = result.getBotpose();
                if (botPoseMt2 != null) {
                    pos = pos.withX(botPoseMt2.getPosition().x);
                    pos = pos.withY(botPoseMt2.getPosition().y);
                }
            }
        }
        return pos;
    }
}