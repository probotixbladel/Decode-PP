package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Configurable
public class Turret {
	private DcMotorEx turretMotor;
	private TouchSensor homeButton;
	private ComponentShell comps;
	private TurretState state = TurretState.HOMING;
	public enum TurretState {
		HOMING,
		SWAPING_LEFT,
		SWAPING_RIGHT,
		CHASING
	}

	public Turret(HardwareMap hwm, ComponentShell compsh) {
		turretMotor = hwm.get(DcMotorEx.class, "turret");
		homeButton = hwm.get(TouchSensor.class, "homer");
		comps = compsh;
	}

	public void update() {
		switch (state) {
			case HOMING:
				home();


		}

	}

	public void home() {
		if (homeButton.isPressed()) {
			turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			state = TurretState.CHASING;
			return;
		}

		if (state == TurretState.HOMING) {
			setPower(0.1);
		}

	}

	private void setPower(double power) {
		if (Math.abs(turretMotor.getPower() - power) > 0.01) {
			turretMotor.setPower(power);
		}
	}

	private double getAnle(double ticks) {
		double motorAngle = ticks / ((1+(46.0 / 11.0)) * 28);
		return 	motorAngle * (12.0 / 130.0);
	}
}
