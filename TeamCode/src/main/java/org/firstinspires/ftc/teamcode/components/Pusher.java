package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Configurable
public class Pusher {
	public Servo Pusher;
    public static double Wait = 0.02; //make higher
    public static double Push = 0.28;
    public ElapsedTime LastShot = new ElapsedTime();

    public PushState state = PushState.RETURNING;
    public static double ShootTime = 0.5;
    public static double ReturnTime = 0.25;
    public static double WaitTime = 0.5;
	public static double MaxSpeed = 0.2;
    public double PusherAngle = 0;
    public static double RestAngle = 335; // make lower
    public static double ArriveAngle = 265; // make higher
    AnalogInput PusherEnc;
    public enum PushState {
        WAITING,
        SHOOTING,
        RELOADING,
        RETURNING
    }

    public Pusher(HardwareMap hwm) {
		Pusher = hwm.get(Servo.class, "Pusher");
        PusherEnc = hwm.get(AnalogInput.class, "PusherEnc");
    }

    public boolean AttemptPush(ComponentShell Comps) {
        if (!Shooter.PreTargeting & Comps.follower.getAngularVelocity() < 0.314 & Comps.follower.getVelocity().getMagnitude() < MaxSpeed) {
            if (state == PushState.WAITING && Comps.shooter.state == Shooter.ShooterState.READY) {
                Pusher.setPosition(Push);
                LastShot.reset();
                state = PushState.SHOOTING;
                return true;
            }
        }
        return false;
    }
    public void update(ComponentShell Comps) {
        PusherAngle = PusherEnc.getVoltage() / 3.3 * 360;
        switch (state) {
            case SHOOTING:
                // PusherAngle > 100 is to avoid detecting overflows
                if (LastShot.seconds() > ShootTime || (PusherAngle < ArriveAngle && PusherAngle > 100)) {
                    Pusher.setPosition(Wait);
                    state = PushState.RETURNING;
                    LastShot.reset();
                }
                break;
            case RETURNING:
                Pusher.setPosition(Wait);
                if (LastShot.seconds() > ReturnTime || PusherAngle > RestAngle) {
                    state = PushState.RELOADING;
                    LastShot.reset();
                }
                break;
            case RELOADING:
                Pusher.setPosition(Wait);
                if(LastShot.seconds() > WaitTime || Comps.detector.detecting) {
                    state = PushState.WAITING;
                }
                break;

        }
    }
}