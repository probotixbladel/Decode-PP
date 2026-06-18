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
    public static double outPower = 1;
    public static double staticPower = 0.7;
    public static double loosenPower = 0;
    public Through.ThroughState state = ThroughState.OFF;
    public enum ThroughState {
        IN_THROUGH,
        OFF
    }

    public Through(HardwareMap hwm) {
		Through = hwm.get(DcMotorEx.class, "Through");
        Through.setDirection(DcMotor.Direction.REVERSE);
        Through.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*public void OutThrough() {
        state = ThroughState.OUT_THROUGH;
        Through.setPower(outPower);
    }

    public void StaticThrough() {
        Through.setPower(staticPower);
        state = ThroughState.OFF;
    }

	public void InThrough() {
		state = ThroughState.IN_THROUGH;
	}*/

	public void update(ComponentShell Comps) {
        if (Comps.intake.state == Intake.IntakeState.OUTTAKE) {
            Through.setPower(outPower);
            return;
        }
        switch (state) {
            case IN_THROUGH:
                if (Comps.pusher.state == Pusher.PushState.WAITING || Comps.pusher.state == Pusher.PushState.RELOADING) {
                    if (Through.getPower() != inPower) {
                        Through.setPower(inPower);
                    }
                    if (Comps.floodgate.floodgateCurrent < 18) {
                        if (Through.getPower() != inPowerOverCurrent) {
                            Through.setPower(inPowerOverCurrent);
                        }
					}
                }
                else {
                    if (Through.getPower() != loosenPower) {Through.setPower(loosenPower);};
                }
                break;
            case OFF:
                Through.setPower(staticPower);
                break;
		}
    }
}