package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Through {
	public DcMotorEx Through;
    public static double inPower = 1.0;
	public static double inPowerOverCurrent = 0.6;
    public static double outPower = -1;
    public static double staticPower = 0;
    public static double loosenPower = -0.2;
    public ThroughState state = ThroughState.OFF;
    public enum ThroughState {
        IN_THROUGH,
        OUT_THROUGH,
        OFF
    }

    public Through(HardwareMap hwm) {
		Through = hwm.get(DcMotorEx.class, "Through");
        Through.setDirection(DcMotor.Direction.REVERSE);
        Through.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void OutThrough() {
        state = ThroughState.OUT_THROUGH;
    }

    public void StaticThrough() {
        state = ThroughState.OFF;
    }

	public void InThrough() {
		state = ThroughState.IN_THROUGH;
	}

	public void update(ComponentShell Comps) {
        switch (state) {
            case IN_THROUGH:
                if (Comps.pusher.state == Pusher.PushState.WAITING || Comps.pusher.state == Pusher.PushState.RELOADING) {
                    Through.setPower(inPower);
					if (Comps.floodgate.floodgateCurrent < 18) {
						Through.setPower(inPowerOverCurrent);
					}
                }
                else {
                    Through.setPower(loosenPower);
                }
                break;
            case OUT_THROUGH:
                Through.setPower(outPower);
                break;
            case OFF:
                Through.setPower(staticPower);
                break;
		}
    }
}