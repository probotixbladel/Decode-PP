package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Configurable
public class Floodgate {
	private final AnalogInput floodgateSens;
    public double floodgateCurrent = 0;
    public Floodgate(HardwareMap hwm) {
		floodgateSens = hwm.get(AnalogInput.class, "FloodgateEnc");

    }
    public void update() {
        floodgateCurrent = floodgateSens.getVoltage() / 3.3 * 80; //80A Max Current
    }
}