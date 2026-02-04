package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Intake {
	public DcMotorEx intake;
	public static double intakePower = 1.0;
	public static double intakePowerOverCurrent = 0.6;
    public static double outtakePower = -0.6;
    public static double staticPower = 0.3;
    public IntakeState state = IntakeState.OFF;
    public enum IntakeState {
        INTAKE,
        OUTTAKE,
        OFF
    }

    public Intake(HardwareMap hwm) {
		intake = hwm.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void TakeIn(ComponentShell Comps) {
		if (Comps.pusher.state == Pusher.PushState.WAITING || Comps.pusher.state == Pusher.PushState.RELOADING) {
			state = IntakeState.INTAKE;
			if (Comps.floodgate.floodgateCurrent > 18) {
				intake.setPower(intakePowerOverCurrent);
			} else {
				intake.setPower(intakePower);
			}
        }
        else {
            intake.setPower(0);
        }
    }

    public void TakeOut() {
        state = Intake.IntakeState.OUTTAKE;
        intake.setPower(outtakePower);
    }
    public void StaticIntake() {
        state = Intake.IntakeState.OFF;
        intake.setPower(staticPower);
    }

}