package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class ArtifactDetector {
    public HardwareMap hardwareMap;
    public DistanceSensor Detector;
    public double distance = 0;
    public double threshold = 20;
    public boolean detecting = false;

    public ArtifactDetector(HardwareMap hwm) {
        this.hardwareMap = hwm;
        this.Detector = hardwareMap.get(DistanceSensor.class, "ColSens");
    }

    public void update() {
        distance = Detector.getDistance(DistanceUnit.MM);
        detecting = distance < threshold;
    }
}
