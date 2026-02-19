package org.firstinspires.ftc.teamcode.components.Shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.common.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.common.RobotMath;
import org.firstinspires.ftc.teamcode.components.ComponentShell;

import java.util.function.DoubleSupplier;

@Configurable
public class Turret {
	private final DcMotorEx turretMotor;
	private final TouchSensor homeButton;
	private final ComponentShell comps;
	private final TurretControlSystems controlSystems;
	private TurretState state = TurretState.HOMING;
	private final RobotMath.AngleTracker targetTracker;
	private Pose[] poseHistory;

	public enum TurretState {
		HOMING,
		SWAPING_LEFT,
		SWAPING_RIGHT,
		CHASING
	}

	public Turret(HardwareMap hwm, ComponentShell componentShell) {
		turretMotor = hwm.get(DcMotorEx.class, "turret");
		homeButton = hwm.get(TouchSensor.class, "homer");
		comps = componentShell;
		targetTracker = new RobotMath.AngleTracker();

		DoubleSupplier motorPosition = () -> getAnleFromTicks(turretMotor.getCurrentPosition());
		DoubleSupplier motorVelocity = () -> getSpeedFromTPS(turretMotor.getVelocity());

		controlSystems = new TurretControlSystems(motorPosition, motorVelocity);
	}

	public void update() {
		targetTracker.addMeasurement(getTargetAngle(), comps.commonData.getTime());
		controlSystems.checkAndUpdate();
		switch (state) {
			case HOMING:
				home();
				break;
			case CHASING:
				setPower(controlSystems.update(getTargetAngle(), targetTracker.getAngularVelocity(), targetTracker.getAngularAcceleration()));

		}
	}

	public void home() {
		if (homeButton.isPressed()) {
			turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			state = TurretState.CHASING;
			return;
		}

		if (state == TurretState.HOMING) {
			setPower(0.1);
		}

	}

	private void setPower(double power) {
		if (Math.abs(turretMotor.getPower() - power) > 0.001) {
			turretMotor.setPower(power);
		}
	}

	private double getAnleFromTicks(double ticks) { //Degrees
		return ticks * TurretConstants.DEGREE_PER_TICK + TurretConstants.ANGLE_OFFSET;
	}

	private double getSpeedFromTPS(double ticksPerSecond) { //Degrees per second
		return ticksPerSecond * TurretConstants.DEGREE_PER_TICK;
	}


	private double getTargetAngle() {
		return Math.min((Math.max(Math.toDegrees(comps.follower.getHeading()), TurretConstants.REACH_MIN)), TurretConstants.REACH_MAX);
	}
}
