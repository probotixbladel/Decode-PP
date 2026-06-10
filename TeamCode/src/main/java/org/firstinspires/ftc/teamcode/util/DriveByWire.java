package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.bylazar.configurables.annotations.Configurable;
import com.ThermalEquilibrium.homeostasis.Systems.PositionVelocitySystem;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.RawValue;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.ComponentShell;

@Configurable
public class DriveByWire {
	// TODO: Tune these values for your application
	// This does NOT create any mechanical advantage, it is purely for control
	private final double[] translationGears = { 0.3, 0.6, 0.8, 1.0 };
	private final double[] rotationGears = { 0.3, 0.4, 0.6, 0.75 };

	public int gear = 3; // the index of the gear in use
	public Double lastAimAngle;
	public double angularVelocity = 0;
	public ElapsedTime angularVelocityTimer = new ElapsedTime();
	public double lastAngularVelocity = 0;

	// Curves exponents
	// higher: more control when slow, less control at speed
	// lower : less control when slow, more control at speed
	// n < 1 or n > 3: not recommended
	// n = 1: linear
	// 1 < n < 3: recommended
	// TODO: experiment to find your optimal value
	private static final double translationCurveExponent = 2.0;
	private static final double rotationCurveExponent = 1.2;

	private final ComponentShell comps;
	public static double FF_KV = 0;
	public static double FF_KA = 0;
	public static double FF_KS = 0;
	public static double VelKP = 0;
	public static double VelKI = 0;
	public static double VelKD = 0;
	public static double PosKP = 0;
	public static double PosKI = 0;
	public static double PosKD = 0;

	public static double offsetX = 1.5748;

	public PositionVelocitySystem aimSystem;
	public DriveByWire(ComponentShell Comps) {
		this.comps = Comps;
	}


	// Applies a signed exponential curve to controller input.
	// This preserves the input direction while adjusting sensitivity.
	private double applyInputCurve(double input, double exponent) {
		return Math.pow(Math.abs(input), exponent) * Math.signum(input);
	}

	// Applies both gearing and input curves to translation & rotation input
	public double[] scaledInput(double x, double y, double yaw) {
		double length = Math.sqrt((x * x + y * y));
		double scale = applyInputCurve(length, translationCurveExponent) * translationGears[gear];
		yaw = applyInputCurve(yaw, rotationCurveExponent) * rotationGears[gear];
		x *= scale;
		y *= scale;
		return new double[] {x, y, yaw};

	}

	public double angleWrap(double radians) {
		while (radians > Math.PI) {
			radians -= 2 * Math.PI;
		}
		while (radians < -Math.PI) {
			radians += 2 * Math.PI;
		}

		return radians;
	}

	public void resetAimSystem() {
		aimSystem = new PositionVelocitySystem(
				new RawValue(() -> this.lastAimAngle),
				new RawValue(() -> this.angularVelocity),
				new BasicFeedforward(new FeedforwardCoefficients(FF_KV, FF_KA, FF_KS)),
				new BasicPID(new PIDCoefficients(PosKP, PosKI, PosKD)),
				new BasicPID(new PIDCoefficients(VelKP, VelKI, VelKD))
		);
	}

	public double[] adjustInputs(double x, double y, double yaw, Gamepad gamepad1) {
		double[] driveInputs = scaledInput(x, y, yaw);

		if (gamepad1.rightBumperWasPressed()) {
			resetAimSystem();
		}


		if(gamepad1.right_bumper) {
			double dy = comps.shooter.ShootTo.getY() - (comps.follower.getPose().getY() - Math.cos(comps.follower.getHeading()) * offsetX);
			double dx = comps.shooter.ShootTo.getX() - (comps.follower.getPose().getX() + Math.sin(comps.follower.getHeading()) * offsetX);
			double alpha = Math.atan2(dy, dx);
			double beta = angleWrap(alpha - Math.PI);
			if (lastAimAngle == null) {
				lastAimAngle = beta;
			}

			if (angularVelocityTimer.seconds() >= 0) {
				angularVelocity = angleWrap(lastAimAngle - beta) / angularVelocityTimer.seconds();

				driveInputs[2] = Math.min(Math.max(
					aimSystem.update(
						comps.follower.getHeading(),
						comps.follower.getAngularVelocity(),
						(lastAngularVelocity - comps.follower.getAngularVelocity()) / angularVelocityTimer.seconds()
					),
					-1),
					1
				);
			} else {
				angularVelocity = 0;
				driveInputs[2] = Math.min(Math.max(
					aimSystem.update(
						comps.follower.getHeading(), 0, 0
					),
					-1),
					1
				);
				aimSystem.update(comps.follower.getHeading(), 0, 0);
			}

			angularVelocityTimer.reset();
		}

		comps.telemetryM.debug("heading error, angular velocity error", angleWrap(lastAimAngle - comps.follower.getHeading()), angularVelocity - comps.follower.getAngularVelocity());

		return driveInputs;
	}
}