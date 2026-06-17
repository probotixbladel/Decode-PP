package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Configurable
public class Pusher {
	public Servo pusher;
    public static double wait = 0.02;
    public static double push = 0.23;
    public ElapsedTime lastShot = new ElapsedTime();
    public PushState state = PushState.RETURNING;
    public static double shootTime = 0.5;
    public static double returnTime = 0.25;
    public static double waitTime = 1;
	public static double maxSpeed = 0.2;
    public double pusherAngle = 0;
    public static double restAngle = 280; // make lower
    public static double arriveAngle = 275; // make higher
    public static double closeAngleTolerance = 30;
    public static double farAngleTolerance = 6;
    AnalogInput pusherEnc;
    public enum PushState {
        WAITING,
        SHOOTING,
        RELOADING,
        RETURNING
    }

    public Pusher(HardwareMap hwm) {
		pusher = hwm.get(Servo.class, "Pusher");
        pusherEnc = hwm.get(AnalogInput.class, "PusherEnc");
    }

    public void forcePush() {
        pusher.setPosition(push);
        lastShot.reset();
        state = PushState.SHOOTING;
    }

    public synchronized boolean AttemptPush(ComponentShell Comps) {
        Pose goal = new Pose();
        switch (Comps.alliance) {
            case RED:
                goal = new Pose(133,135);
                break;
            case BLUE:
                goal = new Pose(11,135);
                break;
        }

        double distance = (Math.sqrt(Math.pow(
                Comps.follower.getPose().getY() - goal.getY(), 2) + Math.pow(Comps.follower.getPose().getX() - goal.getX(), 2)
        )) * 2.54;
        double a = (farAngleTolerance /2 - closeAngleTolerance /2) / (35 - 150);
        double b = closeAngleTolerance /2 - a * 150;
        double angleTolerance = a * pusherAngle + b;

		if (state == PushState.WAITING && Comps.shooter.state == Shooter.ShooterState.READY) {
            pusher.setPosition(push);
            lastShot.reset();
			state = PushState.SHOOTING;
			return true;
        }
        return false;
    }
    public void update(ComponentShell Comps) {
        pusherAngle = pusherEnc.getVoltage() / 3.3 * 360;
        switch (state) {
            case SHOOTING:
                // PusherAngle > 100 is to avoid detecting overflows
                if (lastShot.seconds() > shootTime || (pusherAngle < arriveAngle && pusherAngle > 100)) {
                    if (pusher.getPosition() != wait) {
                        pusher.setPosition(wait);
                    }
                    state = PushState.RETURNING;
                    lastShot.reset();
                }
                break;
            case RETURNING:
                if (pusher.getPosition() != wait) {
                    pusher.setPosition(wait);
                }
                if (lastShot.seconds() > returnTime || pusherAngle > restAngle) {
                    state = PushState.RELOADING;
                    lastShot.reset();
                }
                break;
            case RELOADING:
                if (pusher.getPosition() != wait) {
                    pusher.setPosition(wait);
                }
                if(lastShot.seconds() > waitTime || Comps.detector.firstDetecting) {
                    state = PushState.WAITING;
                }
                break;

        }
    }
}