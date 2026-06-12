package org.firstinspires.ftc.teamcode.components;


import com.bylazar.configurables.annotations.Configurable;
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class ArtifactDetector {
	public OpticalDistanceSensor firstArtifactSensor1;  // Hardware Device Object
	public OpticalDistanceSensor firstArtifactSensor2;  // Hardware Device Object
    public ColorRangeSensor secondArtifactSensor;
    public RevColorSensorV3 thirdArtifactSensor;
    public ModernRoboticsI2cRangeSensor fourthArtifactSensor;
	public double firstDistance = 0;
	public double firstDistance2 = 0;
    private final double[] distanceBuffer = new double[instances];
    private double runningSum = 0;
    private int bufferIndex = 0;
    static public int instances = 5;
    static public double firstMinThreshold = 0.025;
    static public double firstMaxThreshold = 1.5;
    static public double firstMinThreshold2 = 0.04;
    static public double firstMaxThreshold2 = 1.5;
    public ElapsedTime timer;
    public static double thirdDetectingSince = 0;
    public static double fourthDetectingSince = 0;
    public static double secondDistance = 330;
    public static double thirdDistance = 83;
    public static double fourthDistance = 90;
    public static double thirdDetectingTime = 300;
    public static double fourthDetectingTime = 300;
    public boolean firstDetecting = false;
    public boolean secondDetecting = false;
    public boolean thirdDetecting = false;
    public boolean fourthDetecting = false;


    public ArtifactDetector(HardwareMap hwm) {
		this.secondArtifactSensor = hwm.get(ColorRangeSensor.class, "ColThrough");
        this.thirdArtifactSensor  = hwm.get(RevColorSensorV3.class, "ColIn");
        this.fourthArtifactSensor = hwm.get(ModernRoboticsI2cRangeSensor.class, "RangeIn");
		this.firstArtifactSensor1 = hwm.get(OpticalDistanceSensor.class, "DistSens");
		this.firstArtifactSensor2 = hwm.get(OpticalDistanceSensor.class, "DistSens2");
        timer = new ElapsedTime();
    }
    private double getSmoothedDistance1() {
        double newReading = secondArtifactSensor.getDistance(DistanceUnit.MM);
        if (Double.isNaN(newReading) || newReading > 400) newReading = 400; // clamp invalid readings

        runningSum -= distanceBuffer[bufferIndex];
        runningSum += newReading;
        distanceBuffer[bufferIndex] = newReading;
        bufferIndex = (bufferIndex + 1) % distanceBuffer.length;
        return runningSum / distanceBuffer.length;


    }


	public void update(ComponentShell comps) {
        double secondAverage = getSmoothedDistance1();
		firstDistance = firstArtifactSensor1.getLightDetected();
		firstDistance2 = firstArtifactSensor2.getLightDetected();
        firstDetecting = (firstDistance > firstMinThreshold && firstDistance < firstMaxThreshold) || (firstDistance2 > firstMinThreshold2 && firstDistance2 < firstMaxThreshold2);
        secondDetecting = (firstDetecting & secondAverage < secondDistance);

        if (thirdArtifactSensor.getDistance(DistanceUnit.MM) < thirdDistance) {
            if (timer.milliseconds() - thirdDetectingSince > thirdDetectingTime) {
                thirdDetecting = true;
            } else thirdDetecting = secondDetecting;
        } else {
            thirdDetecting = false;
            thirdDetectingSince = timer.milliseconds();
        }

		if (fourthArtifactSensor.getDistance(DistanceUnit.MM) < 1 || fourthArtifactSensor.getDistance(DistanceUnit.MM) == 255) {
			fourthDetecting = false;
			fourthDetectingSince = timer.milliseconds();
		} if (fourthArtifactSensor.getDistance(DistanceUnit.MM) < fourthDistance ) {
            if (timer.milliseconds() - fourthDetectingSince > fourthDetectingTime) {
                fourthDetecting = true;
            } else fourthDetecting = thirdDetecting;
        } else {
            fourthDetecting = false;
            fourthDetectingSince = timer.milliseconds();
        }


		comps.telemetryM.debug(
				"secondArtifact: R, G, B, A, Distance",
				secondArtifactSensor.red(),
				secondArtifactSensor.green(),
				secondArtifactSensor.blue(),
                secondArtifactSensor.alpha(),
                secondArtifactSensor.getDistance(DistanceUnit.MM)

		);
		comps.telemetryM.debug("thirdArtifact: R, G, B, A, Distance, rawOptical",
				thirdArtifactSensor.red(),
				thirdArtifactSensor.green(),
				thirdArtifactSensor.blue(),
				thirdArtifactSensor.alpha(),
                thirdArtifactSensor.getDistance(DistanceUnit.MM),
                thirdArtifactSensor.rawOptical()
		);
		comps.telemetryM.debug("fourthArtifact: mm, raw ultrasonic, raw optical",
				fourthArtifactSensor.getDistance(DistanceUnit.MM),
				fourthArtifactSensor.rawUltrasonic(),
				fourthArtifactSensor.rawOptical()
		);
        comps.telemetryM.debug("detecting: first, second, third, fourth", firstDetecting, secondDetecting, thirdDetecting, fourthDetecting);

    }

}
