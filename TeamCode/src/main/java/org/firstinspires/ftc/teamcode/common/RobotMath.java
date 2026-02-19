package org.firstinspires.ftc.teamcode.common;

import java.util.LinkedList;
import java.util.Queue;

public class RobotMath {
	public static class AngleTracker {
		private final Queue<Double> angles; // in degrees
		private final Queue<Double> times;  // elapsed time in seconds

		public AngleTracker() {
			this.angles = new LinkedList<>();
			this.times = new LinkedList<>();
		}

		public void addMeasurement(double angle, double elapsedTime) {
			angles.add(angle);
			times.add(elapsedTime);

			// 3 is the max we need
			if (angles.size() > 3) {
				angles.poll();
				times.poll();
			}
		}

		public double getAngularVelocity() {
			if (angles.size() < 2) {
				return 0;
			}

			Double[] angleArray = angles.toArray(new Double[0]);
			Double[] timeArray = times.toArray(new Double[0]);

			int n = angleArray.length;
			double dTheta = getAngleDifference(angleArray[n-2], angleArray[n-1]);
			double dt = timeArray[n-1] - timeArray[n-2];

			if (dt == 0) return 0.0;
			return dTheta / dt;
		}

		public double getAngularAcceleration() {
			if (angles.size() < 3) {
				return 0;
			}

			Double[] angleArray = angles.toArray(new Double[0]);
			Double[] timeArray = times.toArray(new Double[0]);

			int n = angleArray.length;

			double angleDifference1 = getAngleDifference(angleArray[n-3], angleArray[n-2]);
			double dt1 = timeArray[n-2] - timeArray[n-3];
			double speed1 = (dt1 == 0) ? 0 : angleDifference1 / dt1; //catch 0 div

			double angleDifference2 = getAngleDifference(angleArray[n-2], angleArray[n-1]);
			double dt2 = timeArray[n-1] - timeArray[n-2];
			double speed2 = (dt2 == 0) ? 0 : angleDifference2 / dt2;

			// Angular acceleration is change in angular velocity over time
			double speedDifference = speed2 - speed1;
			double dt = timeArray[n-1] - timeArray[n-3];

			if (dt == 0) return 0.0;
			return speedDifference / dt;
		}

		private double getAngleDifference(double angle1, double angle2) {
			return angleWrapDeg(angle2 - angle1);
		}
	}



	private double angleWrapDeg(double degrees) {
		int i = 0; // infinite loop prevention
		while (degrees < -180.0 && i < 10) {
			degrees += 360.0;
			i++;
		}

		i = 0;
		while (degrees > 180.0 && i < 10) {
			degrees -= 360.0;
			i++;
		}
		return degrees;
	}

}
