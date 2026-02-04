package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Configurable
public class Floodgate {
	private final AnalogInput floodgateEnc;
    public double floodgateCurrent = 0;
    public Floodgate(HardwareMap hwm) {
		floodgateEnc = hwm.get(AnalogInput.class, "FloodgateEnc");

    }
    public void update() {
        floodgateCurrent = floodgateEnc.getVoltage() / 3.3 * 80; //80A Max Current
    }
}