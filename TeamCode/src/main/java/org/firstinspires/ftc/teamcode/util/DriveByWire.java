package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.bylazar.configurables.annotations.Configurable;
import com.ThermalEquilibrium.homeostasis.Systems.PositionVelocitySystem;

import org.firstinspires.ftc.teamcode.components.ComponentShell;

@Configurable
public class DriveByWire {
	// TODO: Tune these values for your application
	// This does NOT create any mechanical advantage, it is purely for control
	private final double[] translationGears = { 0.3, 0.6, 0.8, 1.0 };
	private final double[] rotationGears = { 0.3, 0.4, 0.6, 0.75 };

	public int gear = 3; // the index of the gear in use


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

	static public double kp = 1;
	static public double kd = 0.08;
	static public double kf = 1;

	public static double offsetX = 1.5748;

	public PIDFController GoalPID = new PIDFController(new PIDFCoefficients(0,0,0,0));

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

	public double[] adjustInputs(double x, double y, double yaw, Gamepad gamepad1) {
		double[] driveInputs = scaledInput(x, y, yaw);

		if (gamepad1.rightBumperWasPressed()) {
			GoalPID = new PIDFController(new PIDFCoefficients(kp, 0, kd, kf));
		}


		if(gamepad1.right_bumper) {
			double dy = comps.shooter.ShootTo.getY() - (comps.follower.getPose().getY() - Math.cos(comps.follower.getHeading()) * offsetX);
			double dx = comps.shooter.ShootTo.getX() - (comps.follower.getPose().getX() + Math.sin(comps.follower.getHeading()) * offsetX);
			double alpha = Math.atan2(dy, dx);
			double beta = alpha - Math.PI;

			GoalPID.setTargetPosition(beta);
			GoalPID.updatePosition(comps.follower.getHeading());
			driveInputs[2] = Math.min(Math.max(GoalPID.run(),-1),1);
		}

		comps.telemetryM.debug("heading error", GoalPID.getError());

		return driveInputs;
	}
}