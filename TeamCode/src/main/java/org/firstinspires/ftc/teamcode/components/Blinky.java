package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Blinky {
    public HardwareMap hardwareMap;
    public RevBlinkinLedDriver Blinky;
    public ElapsedTime detectorTimer;
    static public double detectorTime = 1;
    private boolean wasDetecting = false;
    private boolean timerStarted = false;
    public Blinky(HardwareMap hwm){
        this.hardwareMap = hwm;
        this.Blinky = hardwareMap.get(RevBlinkinLedDriver.class, "Blinkin");
        this.detectorTimer = new ElapsedTime();
    }

    public void update(ComponentShell comps){
        /*if (comps.detector.detecting) {
            Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
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
                Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
            }
            else {
                Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN);
            }
        }
        wasDetecting = comps.detector.detecting;*/
        Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN);
    }
}
