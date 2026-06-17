package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;

import org.firstinspires.ftc.teamcode.components.ComponentShell;

@Configurable
public class DriveByWire {
	// TODO: Tune these values for your application
	// This does NOT create any mechanical advantage, it is purely for control
	private final double[] translationGears = { 0.3, 0.6, 0.8, 1.0 };
	private final double[] rotationGears = { 0.3, 0.4, 0.6, 0.75 };

	public int gear = 3; // the index of the gear in use
	public double lastAimAngle = 0;
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
    public static double CloseKF = 0;
	public static double CloseKP = 1.8;
	public static double CloseKI = 0;
	public static double CloseKD = 0.1;
	public static double PosKP = 0;
	public static double PosKI = 0;
	public static double PosKD = 0;
	public static double offsetX = 1.5748;
    public static double startClosePID = 0;
    public static double levelClosePID = 0;
	public PIDFController aimPosPID;
	public PIDFController aimClosePID;

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
		aimPosPID = new PIDFController(new PIDFCoefficients(PosKP, PosKI, PosKD, CloseKF));
		aimClosePID = new PIDFController(new PIDFCoefficients(CloseKP, CloseKI, CloseKD, CloseKF));
	}

	public double getSteeringValue(double angle) {
		comps.telemetryM.addData("111a", angle);
		double headingError = angleWrap(angle - comps.follower.getHeading());
		comps.telemetryM.addData("111h", headingError);
		aimClosePID.updateError(headingError);
		aimPosPID.updateError(headingError);
        double closeMult = Math.min(Math.max((Math.abs(headingError) - startClosePID) / (startClosePID - levelClosePID), 0), 1);
		comps.telemetryM.addData("111c", closeMult);

		return aimPosPID.run() * (1 - closeMult) + aimClosePID.run() * closeMult;
	}


	public double[] adjustInputs(double x, double y, double yaw, Gamepad gamepad1) {
		double[] driveInputs = scaledInput(x, y, yaw);

		if (gamepad1.rightBumperWasPressed()) {
			resetAimSystem();
		}
		double beta = 0;

		if (gamepad1.right_bumper) {
			comps.telemetryM.addData("111", "39");
			double dy = comps.shooter.ShootTo.getY() - (comps.follower.getPose().getY() - Math.cos(comps.follower.getHeading()) * offsetX);
			double dx = comps.shooter.ShootTo.getX() - (comps.follower.getPose().getX() + Math.sin(comps.follower.getHeading()) * offsetX);
			double alpha = Math.atan2(dy, dx);
			beta = angleWrap(alpha - Math.PI);



			if (angularVelocityTimer.seconds() >= 0) {
				driveInputs[2] = getSteeringValue(beta);
			} else {
				angularVelocity = 0;
				driveInputs[2] = getSteeringValue(beta);
			}
			comps.telemetryM.addData("111 fokiskd", "beta");
			lastAimAngle = angleWrap(beta);
			angularVelocityTimer.reset();
		}
		lastAngularVelocity = comps.follower.getAngularVelocity();


		comps.telemetryM.addData("heading", comps.follower.getHeading());
		comps.telemetryM.addData("target angle", beta);
		comps.telemetryM.addData("angular velocity", comps.follower.getAngularVelocity());
		comps.telemetryM.addData("target angular velocity", angularVelocity);

		return driveInputs;
	}
}