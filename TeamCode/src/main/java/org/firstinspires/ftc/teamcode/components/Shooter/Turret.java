package org.firstinspires.ftc.teamcode.components.Shooter;

import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.ThermalEquilibrium.homeostasis.Systems.PositionVelocitySystem;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.KalmanEstimator;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;

import org.firstinspires.ftc.teamcode.common.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.common.RobotMath;
import org.firstinspires.ftc.teamcode.components.ComponentShell;

import java.lang.annotation.Target;
import java.util.function.DoubleSupplier;

@Configurable
public class Turret {
	private final DcMotorEx turretMotor;
	private final TouchSensor homeButton;
	private final ComponentShell comps;
	private TurretState state = TurretState.HOMING;
	private RobotMath.AngleTracker targetTracker;
	private Pose[] poseHistory;
	//TODO: remove static when tuned
	public static KalmanEstimator positionEstimator;
	public static KalmanEstimator velocityEstimator;
	public static FeedforwardCoefficients feedforwardCoefs;
	public static BasicFeedforward feedforwardController;
	public static PIDCoefficients positionCoefs;
	public static PIDCoefficients velocityCoefs;
	public static BasicPID positionPID;
	public static BasicPID velocityPID;
	public static PositionVelocitySystem controller;

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
		targetTracker = new RobotMath.AngleTracker();

		DoubleSupplier motorPosition = () -> getAnleFromTicks(turretMotor.getCurrentPosition());
		DoubleSupplier motorVelocity = () -> getSpeedFromTPS(turretMotor.getVelocity());

		positionEstimator = new KalmanEstimator(
				motorPosition,
				TurretConstants.POS_KAL_Q,
				TurretConstants.POS_KAL_R,
				TurretConstants.POS_KAL_N
		);

		velocityEstimator = new KalmanEstimator(
				motorVelocity,
				TurretConstants.VEL_KAL_Q,
				TurretConstants.VEL_KAL_R,
				TurretConstants.VEL_KAL_N
		);
		feedforwardCoefs = new FeedforwardCoefficients(
				TurretConstants.FF_KS,
				TurretConstants.FF_KV,
				TurretConstants.FF_KA
		);
		positionCoefs = new PIDCoefficients(
				TurretConstants.POS_PID_KP,
				TurretConstants.POS_PID_KI,
				TurretConstants.POS_PID_KD
		);
		velocityCoefs = new PIDCoefficients(
				TurretConstants.VEL_PID_KP,
				TurretConstants.VEL_PID_KI,
				TurretConstants.VEL_PID_KD
		);

		feedforwardController = new BasicFeedforward(feedforwardCoefs);
		positionPID = new BasicPID(positionCoefs);
		velocityPID = new BasicPID(velocityCoefs);

		controller = new PositionVelocitySystem(
			positionEstimator,
			velocityEstimator,
			feedforwardController,
			positionPID,
			velocityPID
			);
		}

	public void update() {
		targetTracker.addMeasurement(getTargetAngle(), comps.commonData.getTime());
		switch (state) {
			case HOMING:
				home();
				break;
			case CHASING:
				setPower(controller.update(getTargetAngle(), targetTracker.getAngularVelocity(), targetTracker.getAngularAcceleration()));

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
