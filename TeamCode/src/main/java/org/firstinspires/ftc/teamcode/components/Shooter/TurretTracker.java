package org.firstinspires.ftc.teamcode.components.Shooter;
import android.annotation.SuppressLint;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Collections;

/**
 * TurretPIDDebugger
 *
 * Tracks and diagnoses turret controller behavior over time.
 * Call update() every loop iteration, then getDebugString() to display
 * live diagnostics on the Driver Station via telemetry.
 *
 * USAGE:
 *   TurretTracker debugger = new TurretTracker();
 *   // in your loop:
 *   debugger.update(targetAngle, currentAngle, turretVelocity);
 *   telemetry.addLine(debugger.getDebugString(targetAngle, currentAngle, turretVelocity));
 */
public class TurretTracker {

	// -------------------------------------------------------------------------
	// Rolling error history
	// -------------------------------------------------------------------------

	/**
	 * Stores the last N absolute error values (in radians).
	 * Used to compute average error and worst-case percentile.
	 */
	private static final int MAX_ERROR_HISTORY = 4000;
	private final ArrayList<Double> errors = new ArrayList<>();

	// -------------------------------------------------------------------------
	// State carried between update() calls
	// -------------------------------------------------------------------------

	/** The error value from the previous loop iteration. Used to detect sign flips. */
	private double lastError = 0;

	// -------------------------------------------------------------------------
	// Oscillation tracking
	// -------------------------------------------------------------------------

	/**
	 * Counts how many times the error has crossed zero (changed sign).
	 * A high flip count means the turret is oscillating — your D-gain
	 * may be too low or your P-gain too high.
	 */
	private int signFlips = 0;

	/**
	 * The largest overshoot magnitude seen so far (in radians).
	 * Captured at the moment of each sign flip, since that's when
	 * the turret has just crossed through the target.
	 */
	private double maxOvershoot = 0;

	/**
	 * Tracks when the last sign flip occurred (in ms via ElapsedTime).
	 * Used to estimate oscillation frequency.
	 *
	 * FTC CHANGE: was System.nanoTime(); now uses ElapsedTime milliseconds.
	 */
	private double lastFlipTimeMs = 0;

	// -------------------------------------------------------------------------
	// Settle detection
	// -------------------------------------------------------------------------

	/**
	 * The timestamp (ms) when the error first dropped below SETTLE_THRESHOLD
	 * in the current "settling" window. Reset to -1 whenever error spikes back up.
	 */
	private double settleStartMs = -1;

	/**
	 * How long (ms) the turret stayed within SETTLE_THRESHOLD continuously.
	 * Set once stable for >= SETTLE_REQUIRED_MS. -1 if never settled.
	 */
	private double settleTimeMs = -1;

	/**
	 * Error must stay below this angle (0.5 degrees) to be considered "settled".
	 */
	private static final double SETTLE_THRESHOLD = Math.toRadians(0.5);

	/**
	 * How long the error must stay below the threshold before declaring "settled".
	 * 150ms is reasonable; adjust based on your mechanism's inertia.
	 */
	private static final double SETTLE_REQUIRED_MS = 150.0;

	// -------------------------------------------------------------------------
	// FTC ElapsedTime
	// -------------------------------------------------------------------------

	/**
	 * FTC-provided timer. Counts wall-clock time since construction
	 * (or since reset() was called).
	 *
	 * FTC CHANGE: Replaces all System.nanoTime() calls throughout this class.
	 * Use timer.milliseconds() everywhere — no manual nanosecond conversion needed.
	 */
	private final ElapsedTime timer = new ElapsedTime();

	// -------------------------------------------------------------------------
	// Velocity-error heatmap
	// -------------------------------------------------------------------------

	/**
	 * Buckets turret velocity into bins and tracks average error per bin.
	 * Useful for spotting velocity-dependent tracking problems, e.g.:
	 *   "Error is fine at low speed but blows up above 4 rad/s"
	 *
	 * ADVICE: This is a useful diagnostic but you can remove it if telemetry
	 * space is tight. The output is only meaningful after many samples per bin.
	 *
	 * Bin layout (each bin covers 2 rad/s of speed):
	 *   bin 0: 0-2 rad/s
	 *   bin 1: 2-4 rad/s
	 *   bin 2: 4-6 rad/s
	 *   bin 3: 6-8 rad/s
	 *   bin 4: 8+ rad/s
	 */
	private static final int HEATMAP_BINS = 5;

	/** heatmap[bin][0] = sum of errors in that bin, [1] = sum of velocities. */
	private final double[][] heatmap = new double[HEATMAP_BINS][2];

	/** Number of samples that have landed in each velocity bin. */
	private final int[] heatmapCount = new int[HEATMAP_BINS];

	// -------------------------------------------------------------------------
	// NEW: Derivative spike detection
	// -------------------------------------------------------------------------

	/**
	 * NEW FEATURE: Tracks the largest single-loop jump in error (rad/s equivalent).
	 * A huge dError/dt spike usually means encoder noise or a sudden target jump,
	 * not real motion. If this is large, consider filtering your encoder or
	 * smoothing your target angle before feeding it to the PID.
	 */
	private double maxErrorDerivative = 0;

	/** Timestamp of last update call, used to compute dError/dt. */
	private double lastLoopTimeMs = -1;

	// -------------------------------------------------------------------------
	// NEW: Accumulated error (I-term proxy)
	// -------------------------------------------------------------------------

	/**
	 * NEW FEATURE: Running sum of absolute error over all update() calls.
	 * Acts as a rough proxy for how hard your I-term is working.
	 * If this grows very large without settling, you may have I-term windup.
	 * Most useful when you call reset() between distinct PID actions.
	 */
	private double accumulatedError = 0;

	// =========================================================================
	// Public API
	// =========================================================================

	/**
	 * Call once per loop iteration to feed data into the debugger.
	 *
	 * @param targetAngle    Desired turret angle, in radians
	 * @param currentAngle   Measured turret angle, in radians
	 * @param turretVelocity Measured turret angular velocity, in rad/s
	 */
	public void update(double targetAngle, double currentAngle, double turretVelocity) {

		double nowMs    = timer.milliseconds();
		double error    = targetAngle - currentAngle;
		double absError = Math.abs(error);

		// --- Rolling error history -------------------------------------------
		errors.add(absError);
		if (errors.size() > MAX_ERROR_HISTORY)
			errors.remove(0);

		// --- Accumulated error (I-term proxy) ---------------------------------
		accumulatedError += absError;

		// --- Derivative spike detection ---------------------------------------
		if (lastLoopTimeMs >= 0) {
			double dtMs = nowMs - lastLoopTimeMs;
			if (dtMs > 0) {
				// Convert dt from ms to seconds, compute rate of error change
				double errorRate = Math.abs(error - lastError) / (dtMs / 1000.0);
				if (errorRate > maxErrorDerivative)
					maxErrorDerivative = errorRate;
			}
		}
		lastLoopTimeMs = nowMs;

		// --- Sign-flip / oscillation detection --------------------------------
		// A sign flip means the error crossed zero: the turret passed through target
		if (lastError != 0 && Math.signum(error) != Math.signum(lastError)) {

			signFlips++;

			// Overshoot is the error magnitude just BEFORE crossing zero,
			// which tells you how far past the target the turret traveled
			double overshoot = Math.abs(lastError);
			if (overshoot > maxOvershoot)
				maxOvershoot = overshoot;

			lastFlipTimeMs = nowMs;
		}

		// --- Settle detection -------------------------------------------------
		if (absError < SETTLE_THRESHOLD) {

			// Start timing this settle window if we haven't already
			if (settleStartMs < 0)
				settleStartMs = nowMs;

			double stableMs = nowMs - settleStartMs;

			// Once stable long enough, record it (keep updating to track longest)
			if (stableMs >= SETTLE_REQUIRED_MS)
				settleTimeMs = stableMs;

		} else {
			// Error went out of bounds: reset the settle window
			settleStartMs = -1;
		}

		// --- Velocity-error heatmap update ------------------------------------
		double vel = Math.abs(turretVelocity);
		int bin = (int) Math.min(HEATMAP_BINS - 1, vel / 2.0);

		heatmap[bin][0] += absError;
		heatmap[bin][1] += vel;
		heatmapCount[bin]++;

		lastError = error;
	}

	/**
	 * Returns a formatted multi-line string suitable for telemetry.addLine().
	 * Shows all tracked diagnostics in one compact block.
	 *
	 * @param targetAngle    Desired turret angle, in radians
	 * @param currentAngle   Measured turret angle, in radians
	 * @param turretVelocity Measured turret angular velocity, in rad/s
	 */
	public String getDebugString(double targetAngle, double currentAngle, double turretVelocity) {

		if (errors.isEmpty())
			return "TurretPID — No data yet. Call update() first.";

		double error = targetAngle - currentAngle;

		// --- Compute stats over rolling error history -------------------------
		double sum   = 0;
		double worst = 0;

		for (double e : errors) {
			sum += e;
			if (e > worst) worst = e;
		}

		double avg = sum / errors.size();

		// 99th-percentile error: sorts the history and picks near the top.
		// Better than raw worst-case because it ignores one-off spikes.
		ArrayList<Double> sorted = new ArrayList<>(errors);
		Collections.sort(sorted);

		int idx  = Math.min((int)(sorted.size() * 0.99), sorted.size() - 1);
		double p99 = sorted.get(idx);

		// --- Oscillation frequency estimate -----------------------------------
		// Time since last flip is treated as a half-period (one zero crossing = half cycle).
		// ADVICE: This is a rough approximation. For accuracy, store the two most
		// recent flip times and divide 1.0 by their difference.
		double oscHz = 0;
		if (lastFlipTimeMs > 0) {
			double dtMs = timer.milliseconds() - lastFlipTimeMs;
			if (dtMs > 0) oscHz = 1000.0 / dtMs;
		}

		// --- Settle time ------------------------------------------------------
		// settleTimeMs is -1 if we've never been stable for 150ms continuously
		double settleMs = settleTimeMs > 0 ? settleTimeMs : -1;

		// --- Stability score --------------------------------------------------
		// Heuristic 0-1 score: higher = better.
		// Penalizes average error, overshoot, and oscillation frequency.
		// ADVICE: The weights (2, 3, 1) are arbitrary. Tune them to emphasize
		// what matters most for your mechanism.
		double stability = 1.0 / (1.0 + avg * 2 + maxOvershoot * 3 + oscHz);

		// --- Bad-loop flag ----------------------------------------------------
		// True when any metric crosses a threshold worth investigating.
		// ADVICE: Tune these thresholds to your mechanism.
		//   >5 deg overshoot and >6Hz oscillation are red flags for most turrets.
		boolean badLoop =
				maxOvershoot > Math.toRadians(5) ||   // exceeded 5 degrees of overshoot
						oscHz        > 6                   || // oscillating faster than 6 Hz
						avg          > Math.toRadians(2);     // average error above 2 degrees

		// --- Build heatmap string ---------------------------------------------
		// Only shows bins with actual data
		StringBuilder heatmapStr = new StringBuilder();
		for (int i = 0; i < HEATMAP_BINS; i++) {
			if (heatmapCount[i] == 0) continue;
			double avgErr = heatmap[i][0] / heatmapCount[i];
			double avgVel = heatmap[i][1] / heatmapCount[i];
			heatmapStr.append(String.format("v%.1f -> err%.3f  ", avgVel, avgErr));
		}

		// --- Compose output ---------------------------------------------------
		return String.format(
				"=== Turret PID Debug ===\n"               +
						"err: %.3f rad   vel: %.2f rad/s\n"        +
						"avg: %.3f  worst: %.3f  p99: %.3f\n"      +
						"overshoot(max): %.3f rad\n"                +
						"settle: %.1f ms\n"                         +
						"osc: %.2f Hz   flips: %d\n"                +
						"dErr/dt(max): %.2f rad/s\n"                +
						"accum err: %.2f rad\n"                     +
						"stability: %.2f\n"                         +
						"badLoop: %s\n"                             +
						"heatmap: %s",
				error,
				turretVelocity,
				avg,
				worst,
				p99,
				maxOvershoot,
				settleMs,
				oscHz,
				signFlips,
				maxErrorDerivative,
				accumulatedError,
				stability,
				badLoop,
				heatmapStr
		);
	}

	/**
	 * Resets all tracked state and restarts the timer.
	 *
	 * Call this at the start of a new auto segment, or between PID tuning
	 * runs so stale data doesn't pollute your diagnostics.
	 */
	public void reset() {
		errors.clear();
		lastError          = 0;
		signFlips          = 0;
		maxOvershoot       = 0;
		lastFlipTimeMs     = 0;
		settleStartMs      = -1;
		settleTimeMs       = -1;
		maxErrorDerivative = 0;
		accumulatedError   = 0;
		lastLoopTimeMs     = -1;

		for (int i = 0; i < HEATMAP_BINS; i++) {
			heatmap[i][0]  = 0;
			heatmap[i][1]  = 0;
			heatmapCount[i] = 0;
		}

		timer.reset();
	}
}