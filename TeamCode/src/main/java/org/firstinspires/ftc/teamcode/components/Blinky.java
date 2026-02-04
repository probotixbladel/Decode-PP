package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Blinky {
    public HardwareMap hardwareMap;
    public RevBlinkinLedDriver Blinky;
    public ElapsedTime detectorTimer = new ElapsedTime();
	public ElapsedTime strobeTimer;
	public static double strobeTime = 0.2;
    public static double detectorTimeThreshold = 0.6;
	public boolean strobeLawn = false;
	public BlinkState state = BlinkState.IDLE;
	public enum BlinkState {
		WAS_DETECTING,
		DETECTING,
		IDLE
	}

	public Blinky(HardwareMap hwm){
        this.hardwareMap = hwm;
        this.Blinky = hardwareMap.get(RevBlinkinLedDriver.class, "Blinkin");
		this.strobeTimer = new ElapsedTime();
    }

    public void update(ComponentShell comps){
		if (comps.detector.detecting) {
			detectorTimer.reset();
		}

		switch (state) {
			case DETECTING:
				if (detectorTimer.seconds() > detectorTimeThreshold) {
					state = BlinkState.WAS_DETECTING;
				}
				if (strobeTimer.seconds() > strobeTime) {
					strobeTimer.reset();
					strobeLawn = !strobeLawn;
				}

				if (strobeLawn) {
					Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
				} else {
					Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
				}
				break;

			case WAS_DETECTING:
				Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
				if (detectorTimer.seconds() > detectorTimeThreshold + 1.0) {
					state = BlinkState.IDLE;
				}
				if (detectorTimer.seconds() < detectorTimeThreshold) {
					state = BlinkState.DETECTING;
				}
				break;

			case IDLE:
				Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN);
				if (detectorTimer.seconds() < detectorTimeThreshold) {
					state = BlinkState.DETECTING;
				}
		}
    }
}
