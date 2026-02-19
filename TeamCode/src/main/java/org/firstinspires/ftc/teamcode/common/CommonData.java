package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.util.ElapsedTime;

public class CommonData {
	private double dt = 0.1;
	private final ElapsedTime timer;
	private double lastTime = 0;
	private double cycleFrequency = 0;

	public CommonData() {
		timer = new ElapsedTime();
	}

	public void update() {
		dt = lastTime - timer.seconds();
		lastTime = timer.seconds();
		if (dt == 0) {
			dt = 0.0000001;
		}
		cycleFrequency = 1 / dt;
	}

	public void restartTime() {
		timer.reset();
	}

	public double getTime() {
		return timer.seconds();
	}

	public double getCycleFrequency() {
		return cycleFrequency;
	}

	public double getDt() {
		return dt;
	}
}
