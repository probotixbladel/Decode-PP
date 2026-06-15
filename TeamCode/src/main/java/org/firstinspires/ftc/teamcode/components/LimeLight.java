package org.firstinspires.ftc.teamcode.components;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.bylazar.telemetry.TelemetryManager;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;

@Configurable
public class LimeLight {
	public Limelight3A limeLight;
	public static double angleOffset = 0;
    public static double xMul = 39.37;
    public static double yMul = 39.37;
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
        //telemetryM.debug("P Index: ", status.getPipelineIndex(), "P Type: ", status.getPipelineType());

        Pose3D botPoseMt2;
        Pose pos = new Pose();
        limeLight.updateRobotOrientation(robotYaw + angleOffset);
        LLResult result = limeLight.getLatestResult();

        if (result != null && result.isValid()) {
            if (result.getStaleness() < 100) {
                botPoseMt2 = result.getBotpose_MT2();
                if (botPoseMt2 != null) {
                    pos = pos.withX(botPoseMt2.getPosition().x * xMul);
                    pos = pos.withY(botPoseMt2.getPosition().y * yMul);
					pos = pos.withHeading(Math.toRadians(botPoseMt2.getOrientation().getYaw()));
                }
            }
        }
        return new Pose(-pos.getX() + 72, pos.getY() + 72, pos.getHeading());//, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
}