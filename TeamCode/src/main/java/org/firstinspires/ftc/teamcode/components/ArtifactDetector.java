package org.firstinspires.ftc.teamcode.components;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
@Configurable
public class ArtifactDetector {
	public OpticalDistanceSensor firstArtifactSensor1;  // Hardware Device Object
	public OpticalDistanceSensor firstArtifactSensor2;  // Hardware Device Object
    public RevColorSensorV3 secondArtifactSensor;
    public ModernRoboticsI2cRangeSensor thirdArtifactSensor;
    public ModernRoboticsI2cRangeSensor fourthArtifactSensor;
	public double firstDistance = 0;
	public double firstDistance2 = 0;
    static public double firstMinThreshold = 0.025;
    static public double firstMaxThreshold = 1.5;
    static public double firstMinThreshold2 = 0.04;
    static public double firstMaxThreshold2 = 1.5;
    public boolean firstDetecting = false;


    public ArtifactDetector(HardwareMap hwm) {
        //Distance sensor left intake - control hub i2c 3 RangeInlong
        //Distance sensor intake up - control hub i2c 2 RangeInUp
        //ColSens v2 - control hub i2c 0 ColSens
        this.secondArtifactSensor = hwm.get(RevColorSensorV3.class, "ColSens");
        this.thirdArtifactSensor  = hwm.get(ModernRoboticsI2cRangeSensor.class, "RangeInUp");
        this.fourthArtifactSensor = hwm.get(ModernRoboticsI2cRangeSensor.class, "RangeInLong");
		this.firstArtifactSensor1 = hwm.get(OpticalDistanceSensor.class, "DistSens");
		this.firstArtifactSensor2 = hwm.get(OpticalDistanceSensor.class, "DistSens2");
    }

	public void update(ComponentShell Comps) {
		firstDistance = firstArtifactSensor1.getLightDetected();
		firstDistance2 = firstArtifactSensor2.getLightDetected();
        firstDetecting = (firstDistance > firstMinThreshold && firstDistance < firstMaxThreshold) || (firstDistance2 > firstMinThreshold2 && firstDistance2 < firstMaxThreshold2);
    }

}
