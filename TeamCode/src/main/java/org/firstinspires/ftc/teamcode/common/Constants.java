package org.firstinspires.ftc.teamcode.common;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class Constants {
	public static class GeneralConstants {
		public static Pose GOAL_BLUE = new Pose(3, 141);
		public static Pose GOAL_RED = new Pose(141, 141);
	}
	public static class CurveConstants {
		public static int MAX_POINTS = 6;
		public static double RECENCY_BIAS = 0.3;
	}
}
