package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;





public class ArtifactDetector {
    public HardwareMap hardwareMap;
    public OpticalDistanceSensor odsSensor;  // Hardware Device Object
    public RevBlinkinLedDriver Blinkin;
    public double distance = 0;
    public double threshold = 0.025;
    public boolean detecting = false;
    public ElapsedTime detectorTimer;
    static public double detectorTime = 1;
    private boolean wasDetecting = false;
    private boolean timerStarted = false;


    public ArtifactDetector(HardwareMap hwm) {
        this.hardwareMap = hwm;
        this.odsSensor = hardwareMap.get(OpticalDistanceSensor.class, "DistSens");
        this.Blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "Blinkin");
        this.detectorTimer = new ElapsedTime();
    }

    public void update() {
        distance = odsSensor.getLightDetected();
        detecting = distance > threshold;


        if (detecting) {
            Blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
            if (!wasDetecting) {
                detectorTimer.reset();
                timerStarted = false;
            }
        } else {
            if (wasDetecting && !timerStarted) {
                detectorTimer.reset();
                timerStarted = true;
            }
            if (timerStarted && detectorTimer.seconds() < detectorTime) {
                Blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
            }
            else {
                Blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN);
            }
        }
        wasDetecting = detecting;
    }

}
