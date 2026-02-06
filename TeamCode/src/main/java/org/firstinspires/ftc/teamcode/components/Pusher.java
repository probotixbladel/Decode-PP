package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Configurable
public class Pusher {
	public Servo pusher;
    public static double wait = 0.035; //make higher
    public static double push = 0.28;
    public ElapsedTime lastShot = new ElapsedTime();

    public PushState state = PushState.RETURNING;
    public static double shootTime = 0.5;
    public static double returnTime = 0.25;
    public static double waitTime = 1;
	public static double maxSpeed = 0.2;
    public double pusherAngle = 0;
    public static double restAngle = 280; // make lower
    public static double arriveAngle = 300; // make higher
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

    public boolean AttemptPush(ComponentShell Comps) {
        if (!Shooter.PreTargeting & Comps.follower.getAngularVelocity() < 0.314 & Comps.follower.getVelocity().getMagnitude() < maxSpeed) {
            if (state == PushState.WAITING && Comps.shooter.state == Shooter.ShooterState.READY) {
                pusher.setPosition(push);
                lastShot.reset();
                state = PushState.SHOOTING;
                return true;
            }
        }
        return false;
    }
    public void update(ComponentShell Comps) {
        pusherAngle = pusherEnc.getVoltage() / 3.3 * 360;
        switch (state) {
            case SHOOTING:
                // PusherAngle > 100 is to avoid detecting overflows
                if (lastShot.seconds() > shootTime || (pusherAngle < arriveAngle && pusherAngle > 100)) {
                    pusher.setPosition(wait);
                    state = PushState.RETURNING;
                    lastShot.reset();
                }
                break;
            case RETURNING:
                pusher.setPosition(wait);
                if (lastShot.seconds() > returnTime || pusherAngle > restAngle) {
                    state = PushState.RELOADING;
                    lastShot.reset();
                }
                break;
            case RELOADING:
                pusher.setPosition(wait);
                if(lastShot.seconds() > waitTime || Comps.detector.detecting) {
                    state = PushState.WAITING;
                }
                break;

        }
    }
}