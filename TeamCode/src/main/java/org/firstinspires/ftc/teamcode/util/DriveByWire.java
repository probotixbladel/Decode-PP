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
	public static double FF_KV = 0;
	public static double FF_KA = 0.05;
	public static double FF_KS = 0;
	public static double VelKP = 0;
	public static double VelKI = 0;
	public static double VelKD = 0;
	public static double PosKP = 0;
	public static double PosKI = 0;
	public static double PosKD = 0;
	public static double offsetX = 1.5748;
    public static double brakeVelScaler = 0;
    public static double brakeDistScaler = 0;
	public PIDFController aimPosPID;
	public PIDFController aimVelPID;

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
		aimPosPID = new PIDFController(new PIDFCoefficients(PosKP, PosKI, PosKD, 0));
		aimVelPID = new PIDFController(new PIDFCoefficients(VelKP, VelKI, VelKD, 0));
	}

	public double getSteeringValue(double angle, double angularVelocity, double angularAcceleration) {
		double headingError = angleWrap(angle - comps.follower.getHeading());
		aimVelPID.updateError(angularVelocity - comps.follower.getAngularVelocity());
		aimPosPID.updateError(headingError);
		double feedForward = Math.copySign(FF_KS, headingError) + angularVelocity * FF_KV + angularAcceleration * FF_KA;
		return aimPosPID.run() + aimVelPID.run() + feedForward;
	}

    public double[] bindToTriangle(double x, double y, double yaw) {
        double[] translationInputs = {x, y, yaw};
        double leftBrakeDist  = Math.max(0, comps.follower.getVelocity().projectOnto(new Vector(new Pose(-1, -1))).getMagnitude());
        double rightBrakeDist = Math.max(0, comps.follower.getVelocity().projectOnto(new Vector(new Pose(1, -1))).getMagnitude());
        double leftDist  = Math.abs(-72 * comps.follower.getPose().getX() - 72 * comps.follower.getPose().getY() + 10.368) / Math.sqrt(-72 * -72 + -72 * -72);
        double rightDist = Math.abs(-72 * comps.follower.getPose().getX() + 72 * comps.follower.getPose().getY()) / Math.sqrt(-72 * -72 + -72 * -72);
        double leftMagnitude  = (leftDist  * brakeDistScaler) * (brakeVelScaler / leftBrakeDist );
        double rightMagnitude = (rightDist * brakeDistScaler) * (brakeVelScaler / rightBrakeDist);
        Vector stickVector = new Vector(new Pose(x, y)).plus(new Vector(new Pose(1,1)).times(leftMagnitude)).plus(new Vector(new Pose(-1,1)).times(rightMagnitude));
        translationInputs[0] = stickVector.getXComponent();
        translationInputs[1] = stickVector.getYComponent();
        return translationInputs;
    }

	public double[] adjustInputs(double x, double y, double yaw, Gamepad gamepad1) {
		double[] driveInputs = scaledInput(x, y, yaw);

		if (gamepad1.rightBumperWasPressed()) {
			resetAimSystem();
		}

		if(gamepad1.right_bumper & gamepad1.right_trigger > 0.2) {
			double dy = comps.shooter.ShootTo.getY() - (comps.follower.getPose().getY() - Math.cos(comps.follower.getHeading()) * offsetX);
			double dx = comps.shooter.ShootTo.getX() - (comps.follower.getPose().getX() + Math.sin(comps.follower.getHeading()) * offsetX);
			double alpha = Math.atan2(dy, dx);
			double beta = angleWrap(alpha - Math.PI);

			if (angularVelocityTimer.seconds() >= 0) {
				angularVelocity = angleWrap(lastAimAngle - beta) / angularVelocityTimer.seconds();
				driveInputs[2] = getSteeringValue(beta, angularVelocity, (lastAngularVelocity - comps.follower.getAngularVelocity()) / angularVelocityTimer.seconds());
			} else {
				angularVelocity = 0;
				driveInputs[2] = getSteeringValue(beta, angularVelocity, 0);
			}
			angularVelocityTimer.reset();
		}
		lastAngularVelocity = comps.follower.getAngularVelocity();
		lastAimAngle = angleWrap(comps.follower.getHeading());

        if (gamepad1.right_trigger > 0.2) {
            if (comps.follower.getPose().getX() < 72) {
                if (72 - comps.follower.getPose().getX() < comps.follower.getPose().getY() - 72) {
                    driveInputs = bindToTriangle(driveInputs[0], driveInputs[1], driveInputs[2]);
                }
            } else {
                if (comps.follower.getPose().getX() < comps.follower.getPose().getY()) {
                    driveInputs = bindToTriangle(driveInputs[0], driveInputs[1], driveInputs[2]);
                }
            }
        }


		comps.telemetryM.debug(
                "heading error, angular velocity error",
                angleWrap(lastAimAngle - comps.follower.getHeading()),
                angularVelocity - comps.follower.getAngularVelocity()
        );

		return driveInputs;
	}
}