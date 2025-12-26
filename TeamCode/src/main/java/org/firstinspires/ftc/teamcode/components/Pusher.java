package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Configurable
public class Pusher {
    private final HardwareMap hardwareMap;
    public Servo Pusher;
    public static double Wait = 0.0;
    public static double Push = 0.25;
    public ElapsedTime LastShot = new ElapsedTime();

    public PushState state = PushState.RETURNING;
    public static double ShootTime = 0.5;
    public static double ReturnTime = 0.25;
    public double PusherAngle = 0;
    public static double RestAngle = 330;
    public static double AriveAngle = 270;
    AnalogInput PusherEnc;
    public enum PushState {
        WAITING,
        SHOOTING,
        RETURNING
    }

    public Pusher(HardwareMap hwm) {
        this.hardwareMap = hwm;
        Pusher = hardwareMap.get(Servo.class, "Pusher");
        PusherEnc = hardwareMap.get(AnalogInput.class, "PusherEnc");

    }

    public void AttemptPush(ComponentShell Comps) {
        if (state == PushState.WAITING & Comps.shooter.state == Shooter.ShooterState.READY) {
            Pusher.setPosition(Push);
            state = PushState.SHOOTING;
            LastShot.reset();
        }
    }
    public void update(ComponentShell Comps) {
        PusherAngle = PusherEnc.getVoltage() / 3.3 * 360;
        switch (state) {
            case SHOOTING:
                if (LastShot.seconds() > ShootTime || PusherAngle < AriveAngle) {
                    Pusher.setPosition(Wait);
                    state = PushState.RETURNING;
                }
            case RETURNING:
                if (LastShot.seconds() > ShootTime + ReturnTime) {
                    state = PushState.WAITING;
                }
        }
    }

}