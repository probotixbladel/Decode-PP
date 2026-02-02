package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Blinky {
    public HardwareMap hardwareMap;
    public RevBlinkinLedDriver Blinky;
    public ElapsedTime detectorTimer;
	public ElapsedTime strobeTimer;
	public static double strobeTime = 0.3;
    public static double detectorTimeTreshold = 0.3;
	public static double colorInterval = 0.05;
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
		if (detectorTimer.seconds() < detectorTimeTreshold) {
			state = BlinkState.DETECTING;
		}

		switch (state) {
			case WAS_DETECTING:
				Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
				if (detectorTimer.seconds() > detectorTimeTreshold + strobeTime) {
					state = BlinkState.IDLE;
				}
				break;
			case DETECTING:
				if (detectorTimer.seconds() > detectorTimeTreshold) {
					state = BlinkState.WAS_DETECTING;
				}
				double relativeSpeed;
				switch (comps.shooter.state) {
					case LOW:
						relativeSpeed = comps.shooter.CurrentVel / Shooter.MinSpeed;
						if (relativeSpeed >= 1.0 - colorInterval) {
							Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
						} else if (relativeSpeed >= 1.0 - colorInterval * 2) {
							Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
						} else if (relativeSpeed >= 1.0 - colorInterval * 3) {
							Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
						} else if (relativeSpeed >= 1.0 - colorInterval * 4) {
							Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
						} else if (relativeSpeed >= 1.0 - colorInterval * 5) {
							Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
						} else {
							Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);
						}
						break;
					case READY:
						if (strobeTimer.seconds() > strobeTime) {
							strobeTimer.reset();
							strobeLawn = !strobeLawn;
						}

						if (strobeLawn) {
							Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN);
						} else {
							Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
						}
						break;
					case HIGH:
						relativeSpeed = comps.shooter.CurrentVel / Shooter.MaxSpeed;
						if (relativeSpeed <= 1.0 + colorInterval) {
							Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
						} else if (relativeSpeed <= 1.0 + colorInterval * 2) {
							Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);
						} else if (relativeSpeed <= 1.0 + colorInterval * 3) {
							Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
						} else if (relativeSpeed <= 1.0 + colorInterval * 4) {
							Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
						} else if (relativeSpeed <= 1.0 + colorInterval * 5) {
							Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
						} else {
							Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
						}
						break;
				}
				break;
			case IDLE:
				Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN);
		}
    }
}
