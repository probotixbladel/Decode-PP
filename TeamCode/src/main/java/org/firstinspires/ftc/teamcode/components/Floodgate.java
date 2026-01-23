package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Configurable
public class Floodgate {
    private final HardwareMap hardwareMap;
    private AnalogInput FloodgateEnc;
    public static double FloodgateCurrent = 0;
    public Floodgate(HardwareMap hwm) {
        this.hardwareMap = hwm;
        FloodgateEnc = hardwareMap.get(AnalogInput.class, "FloodgateEnc");

    }
    public void update(ComponentShell comps) {
        FloodgateCurrent = FloodgateEnc.getVoltage() / 3.3 * 80; //Max Current

    }
}