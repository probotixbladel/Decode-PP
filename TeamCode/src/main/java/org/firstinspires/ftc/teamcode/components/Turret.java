package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Turret {
	private DcMotorEx turretMotor;

	public Turret(HardwareMap hwm) {
		turretMotor = hwm.get(DcMotorEx.class, "turret");
	}

	public void update() {

	}

	private double getAnle(double ticks) {
		double motorAngle = ticks / ((1+(46.0 / 11.0)) * 28);
		return 	motorAngle * (12.0 / 130.0);
	}
}
