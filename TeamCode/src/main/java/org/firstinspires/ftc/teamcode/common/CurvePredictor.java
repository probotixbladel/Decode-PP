package org.firstinspires.ftc.teamcode.common;

import org.firstinspires.ftc.teamcode.common.Constants.CurveConstants;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Objects;


class CurvePredictor {
	private Deque<double[]> points;
	private double recencyBias;
	private int maxPoints;
	private double[] coeffs; // [a, b, c]
	private List<Double> weights;
	private List<Double> weightPowers;

	public CurvePredictor(int maxPoints, double recencyBias) {
		this.maxPoints = maxPoints;
		this.recencyBias = recencyBias;
		this.points = new ArrayDeque<>(maxPoints);
		this.coeffs = null;
		this.weights = new ArrayList<>();
		this.weightPowers = new ArrayList<>();
		precomputeWeightPowers();
	}

	private void precomputeWeightPowers() {
		weightPowers.clear();
		for (int i = 0; i < maxPoints; i++) {
			weightPowers.add(Math.pow(recencyBias, i));
		}
	}

	private void updateWeights() {
		int n = points.size();
		weights.clear();
		if (n == 0) return;

		Double[] powers = new Double[n];
		int idx = n - 1;
		for (double w : weightPowers) {
			if (idx < 0) break;
			powers[idx--] = w;
		}

		// Reverse order: newest point gets highest weight
		double total = 0.0;
		for (int i = 0; i < n; i++) {
			weights.add(powers[i]);
			total += powers[i];
		}

		// Normalize
		for (int i = 0; i < n; i++) {
			weights.set(i, weights.get(i) / total);
		}
	}

	public boolean addPoint(double x, double y) {
		if (points.isEmpty() || x > Objects.requireNonNull(points.peekLast())[0]) {
			if (points.size() == maxPoints) points.pollFirst();
			points.addLast(new double[]{x, y});
			return true;
		}
		return false;
	}

	public double[] getCoeffs() {
		computeCoefficients();
		return coeffs;
	}

	public List<Double> predict(List<Double> xValues) {
		computeCoefficients();
		if (coeffs == null) return null;

		List<Double> yValues = new ArrayList<>();
		for (double x : xValues) {
			yValues.add(coeffs[0] * x * x + coeffs[1] * x + coeffs[2]);
		}
		return yValues;
	}

	private void computeCoefficients() {
		int n = points.size();
		if (n == 0) {
			coeffs = null;
			return;
		}

		updateWeights();
		List<Double> xVals = new ArrayList<>();
		List<Double> yVals = new ArrayList<>();
		for (double[] p : points) {
			xVals.add(p[0]);
			yVals.add(p[1]);
		}

		if (n == 1) {
			coeffs = new double[]{0.0, 0.0, yVals.get(0)};
		} else if (n == 2) {
			double x1 = xVals.get(0), y1 = yVals.get(0);
			double x2 = xVals.get(1), y2 = yVals.get(1);
			if (Math.abs(x2 - x1) < 1e-10) {
				coeffs = new double[]{0.0, 0.0, (weights.get(0)*y1 + weights.get(1)*y2)};
			} else {
				double b = (y2 - y1) / (x2 - x1);
				double c = y1 - b * x1;
				coeffs = new double[]{0.0, b, c};
			}
		} else {
			// Quadratic: solve 3x3 weighted normal equation
			double[][] XtWX = new double[3][3];
			double[] XtWy = new double[3];

			for (int i = 0; i < n; i++) {
				double x = xVals.get(i);
				double y = yVals.get(i);
				double w = weights.get(i);
				double x2 = x*x, x3 = x2*x, x4 = x2*x2;

				XtWX[0][0] += w * x4; XtWX[0][1] += w * x3; XtWX[0][2] += w * x2;
				XtWX[1][0] += w * x3; XtWX[1][1] += w * x2; XtWX[1][2] += w * x;
				XtWX[2][0] += w * x2; XtWX[2][1] += w * x;  XtWX[2][2] += w;

				XtWy[0] += w * x2 * y;
				XtWy[1] += w * x * y;
				XtWy[2] += w * y;
			}

			coeffs = solve3x3(XtWX, XtWy);
			if (coeffs == null) {
				// fallback to linear
				double x1 = xVals.get(n-2), y1 = yVals.get(n-2);
				double x2 = xVals.get(n-1), y2 = yVals.get(n-1);
				if (Math.abs(x2 - x1) >= 1e-10) {
					double b = (y2 - y1)/(x2 - x1);
					double c = y1 - b*x1;
					coeffs = new double[]{0.0, b, c};
				} else {
					coeffs = new double[]{0.0, 0.0, y2};
				}
			}
		}
	}

	// Gaussian elimination with partial pivoting
	private double[] solve3x3(double[][] matrix, double[] vector) {
		double[][] aug = new double[3][4];
		for (int i = 0; i < 3; i++) {
			System.arraycopy(matrix[i], 0, aug[i], 0, 3);
			aug[i][3] = vector[i];
		}

		// Forward elimination
		for (int i = 0; i < 3; i++) {
			// Find pivot
			int maxRow = i;
			for (int k = i + 1; k < 3; k++) {
				if (Math.abs(aug[k][i]) > Math.abs(aug[maxRow][i])) maxRow = k;
			}
			double[] tmp = aug[i]; aug[i] = aug[maxRow]; aug[maxRow] = tmp;

			if (Math.abs(aug[i][i]) < 1e-12) return null;

			for (int k = i + 1; k < 3; k++) {
				double factor = aug[k][i]/aug[i][i];
				for (int j = i; j < 4; j++) {
					aug[k][j] -= factor * aug[i][j];
				}
			}
		}

		// Back substitution
		double[] sol = new double[3];
		for (int i = 2; i >= 0; i--) {
			sol[i] = aug[i][3];
			for (int j = i+1; j < 3; j++) sol[i] -= aug[i][j]*sol[j];
			sol[i] /= aug[i][i];
		}
		return sol;
	}

	// Additional methods
	public void clear() {
		points.clear();
		weights.clear();
		coeffs = null;
	}

	public void updateRecencyBias(double newBias) {
		recencyBias = newBias;
		precomputeWeightPowers();
	}

	public void updateMaxPoints(int newMax) {
		maxPoints = newMax;
		Deque<double[]> newPoints = new ArrayDeque<>(maxPoints);
		int skip = Math.max(0, points.size() - maxPoints);
		int count = 0;
		for (double[] p : points) {
			if (count++ >= skip) newPoints.add(p);
		}
		points = newPoints;
		precomputeWeightPowers();
	}
}
