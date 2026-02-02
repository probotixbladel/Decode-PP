package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;





public class ArtifactDetector {
    public HardwareMap hardwareMap;
    public OpticalDistanceSensor odsSensor;  // Hardware Device Object

    public double distance = 0;
    public double threshold = 0.025;
    public boolean detecting = false;


    public ArtifactDetector(HardwareMap hwm) {
        this.hardwareMap = hwm;
        this.odsSensor = hardwareMap.get(OpticalDistanceSensor.class, "DistSens");
    }

    public void update() {
        distance = odsSensor.getLightDetected();
        detecting = distance > threshold;



    }

}
