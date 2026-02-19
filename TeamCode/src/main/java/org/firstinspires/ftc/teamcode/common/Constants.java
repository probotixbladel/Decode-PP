package org.firstinspires.ftc.teamcode.common;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class Constants {
	public static class GeneralConstants {
		public static boolean DEBUG = false;
		public static Pose GOAL_BLUE = new Pose(3, 141);
		public static Pose GOAL_RED = new Pose(141, 141);
	}
	public static class CurveConstants {
		public static int MAX_POINTS = 6;
		public static double RECENCY_BIAS = 0.3;
	}

	public static class TurretConstants {
		public static double ENCODER_RESOLUTION = (1 + (46.0 / 11.0)) * 28;
		public static double DEGREE_PER_TICK = (ENCODER_RESOLUTION * (12.0 / 130.0)) / 360.0;
		public static double ANGLE_OFFSET = 0;
		public static double REACH_MAX = 90;
		public static double REACH_MIN = -90;
		public static double POS_KAL_Q = 1;
		public static double POS_KAL_R = 1;
		public static int POS_KAL_N = 1;
		public static double VEL_KAL_Q = 1;
		public static double VEL_KAL_R = 1;
		public static int VEL_KAL_N = 1;
		public static double FF_KS = 0;
		public static double FF_KV = 0;
		public static double FF_KA = 0;
		public static double POS_PID_KP = 0;
		public static double POS_PID_KI = 0;
		public static double POS_PID_KD = 0;
		public static double VEL_PID_KP = 0;
		public static double VEL_PID_KI = 0;
		public static double VEL_PID_KD = 0;
	}
}
