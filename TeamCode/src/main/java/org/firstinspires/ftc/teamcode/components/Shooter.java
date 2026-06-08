package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;


@Configurable
public class Shooter {
    public static double airTimeMultiplier = 0.4;
	public DcMotorEx FlyWheel;
    public DcMotorEx AntiBackspin;
    //public DcMotorEx ShooterRight;
    public ShooterState state = ShooterState.LOW;
    public static double TargetVel = 1250;
    public static double MaxSpeed = 1300;
    public static double MinSpeed = 1200;
    public double CurrentVel = 0;
    public boolean PreTargeting = false;
    public static double[][] MinPoints = {  // data min speed
            {60,670},
            {70,670},
            {80,640},
            {90,620},
            {100,620},
            {110,630},
            {120,630},
            {130,630},
            {140,640},
            {150,660},
            {160,670},
            {170,700},
            {180,710},
            {190,730},
            {200,750},
            {210,770},
            {220,780},
            {230,800},
            {240,840},
            {250,860},
            {260,880},
            {270,900},
            {280,910},
            {290,940},
            {300,950},
            {310,970},
            {320,1000},
            {330,1010},
            {340,1010},
            {350,1020},
            {360,1030}
    };

    public static double[][] MaxPoints = { // data max speed
            {60,750},
            {70,760},
            {80,750},
            {90,750},
            {100,740},
            {110,750},
            {120,750},
            {130,750},
            {140,760},
            {150,790},
            {160,780},
            {170,790},
            {180,790},
            {190,840},
            {200,860},
            {210,870},
            {220,870},
            {230,900},
            {240,920},
            {250,930},
            {260,960},
            {270,980},
            {280,990},
            {290,1000},
            {300,990},
            {310,1010},
            {320,1070},
            {330,1070},
            {340,1070},
            {350,1090},
            {360,1120}
    };
    public static double P = 500.0;
    public static double D = 0.0;
    public static double F = 14.0;
    private double lP = P;
    private double lD = D;
    private double lF = F;
    public static double MinToMax = 0.7; //percentage min-max
    public Pose ShootTo;
	private ComponentShell.Alliance alliance;
    public enum ShooterState {
        READY,
        HIGH,
        LOW,
    }

    public Shooter(HardwareMap hwm, ComponentShell.Alliance al) {
        FlyWheel = hwm.get(DcMotorEx.class, "flyWheel");
        FlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        FlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FlyWheel.setVelocityPIDFCoefficients(P, 0, D, F);

        AntiBackspin = hwm.get(DcMotorEx.class, "antiBackspin");
        AntiBackspin.setDirection(DcMotorSimple.Direction.FORWARD);
        AntiBackspin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        AntiBackspin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        AntiBackspin.setVelocityPIDFCoefficients(P, 0, D, F);
		alliance = al;
        switch (al) {
            case RED:
                ShootTo = new Pose(133,135);
                break;
            case BLUE:
                ShootTo = new Pose(11,135);
                break;
        }
    }

    public static double interpolate(double[][] p, double x) {
        int i = 0;

        // Find the segment containing x
        while (i < p.length - 1 && x > p[i + 1][0]) i++;

        // Clamp to bounds
        if (i >= p.length - 1) return p[p.length - 1][1];
        if (x <= p[0][0]) return p[0][1];

        double x0 = p[i][0];
        double y0 = p[i][1];
        double x1 = p[i + 1][0];
        double y1 = p[i + 1][1];

        // Linear interpolation
        return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
    }

	public void updateShootTo(ComponentShell comps) {
		Pose goal = new Pose();
		switch (alliance) {
			case RED:
				goal = new Pose(133,135);
				break;
			case BLUE:
				goal = new Pose(11,135);
				break;
		}

		Vector goalVec = new Vector(goal);
		double airTime;
		this.setSpeeds(comps.follower.getPose());
		for (int i = 0; i < 3; i++) {
			airTime = -3.74 + 8.89e-3 * TargetVel - 4.12e-6 * Math.pow(TargetVel, 2);
			goalVec = goalVec.minus(comps.follower.getVelocity().times(airTime * airTimeMultiplier));
			ShootTo = new Pose(goalVec.getXComponent(), goalVec.getYComponent());
			this.setSpeeds(comps.follower.getPose());
			comps.telemetryM.debug("air time, goalVec, shootTo, targetVel", airTime, goalVec, ShootTo, TargetVel, comps.follower.getVelocity());
		}
	}

    public void setSpeeds(Pose robotPose) {
        double distance = (Math.sqrt(Math.pow(robotPose.getY() - ShootTo.getY(), 2) + Math.pow(robotPose.getX() - ShootTo.getX(), 2)) - 8) * 2.54;
        MaxSpeed = interpolate(MaxPoints, distance) - 5;
        MinSpeed = interpolate(MinPoints, distance) + 5;
        MinToMax = -5.05e-4 * distance + 0.945;
        TargetVel = MinSpeed + (MaxSpeed - MinSpeed) * MinToMax;
	}

    public void PreTargetTo(Pose RobotPos) {
        setSpeeds(RobotPos);
        PreTargeting = true;
    }

    public void Arrived() {
        PreTargeting = false;
    }

    public void update(ComponentShell Comps){
		this.updateShootTo(Comps);
        if (P != lP | D != lD | F != lF){
            FlyWheel.setVelocityPIDFCoefficients( P, 0, D, F);
            AntiBackspin.setVelocityPIDFCoefficients( P, 0, D, F);
            lP = P;
            lD = D;
            lF = F;
        }

        FlyWheel.setVelocity(TargetVel);
        AntiBackspin.setVelocity(TargetVel);
        CurrentVel = FlyWheel.getVelocity();
        if (CurrentVel < MinSpeed) {
            state = ShooterState.LOW;
            //state = ShooterState.READY;
        } else if (CurrentVel > MaxSpeed) {
            state = ShooterState.HIGH;
            //state = ShooterState.READY;
        } else {
            state = ShooterState.READY;
        }
    }

}
