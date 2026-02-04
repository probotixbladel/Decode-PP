package org.firstinspires.ftc.teamcode.components;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
@Configurable
public class ArtifactDetector {
	public OpticalDistanceSensor odsSensor;  // Hardware Device Object
	public OpticalDistanceSensor odsSensor2;  // Hardware Device Object

	public double distance = 0;
	public double distance2 = 0;
    static public double minThreshold = 0.025;
    static public double maxThreshold = 2.0;
    public boolean detecting = false;


    public ArtifactDetector(HardwareMap hwm) {
		this.odsSensor = hwm.get(OpticalDistanceSensor.class, "DistSens");
		this.odsSensor2 = hwm.get(OpticalDistanceSensor.class, "DistSens2");
    }

	public void update() {
		distance = odsSensor.getLightDetected();
		distance2 = odsSensor2.getLightDetected();
        detecting = (distance > minThreshold && distance < maxThreshold) || (distance2 > minThreshold && distance2 < maxThreshold);
    }

}
