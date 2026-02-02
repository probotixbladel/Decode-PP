package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Blinky {
    public HardwareMap hardwareMap;
    public RevBlinkinLedDriver Blinky;
    public ElapsedTime detectorTimer;
	public ElapsedTime strobeTimer;
	public static double strobeTime = 0.2;
    public static double detectorTimeTreshold = 0.6;
	public static double colorInterval = 0.02;
	public boolean strobeLawn = false;
    private boolean detecting = false;
	private boolean wasDetecting = false;
	public BlinkState state = BlinkState.IDLE;
	public enum BlinkState {
		WAS_DETECTING,
		DETECTING,
		IDLE
	}

    public Blinky(HardwareMap hwm){
        this.hardwareMap = hwm;
        this.Blinky = hardwareMap.get(RevBlinkinLedDriver.class, "Blinkin");
        this.detectorTimer = new ElapsedTime();
		this.strobeTimer = new ElapsedTime();
    }

    public void update(ComponentShell comps){
		if (comps.detector.detecting) {
			detectorTimer.reset();
		}

		switch (state) {
			case DETECTING:
				if (detectorTimer.seconds() > detectorTimeTreshold) {
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
				if (detectorTimer.seconds() > detectorTimeTreshold + 1.0) {// dis goeie
					state = BlinkState.IDLE;
				}
				if (detectorTimer.seconds() < detectorTimeTreshold) {
					state = BlinkState.DETECTING;
				}
				break;

			case IDLE:
				Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN);
				if (detectorTimer.seconds() < detectorTimeTreshold) {
					state = BlinkState.DETECTING;
				}
		}
    }
}
