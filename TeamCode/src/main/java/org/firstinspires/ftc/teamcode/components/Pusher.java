package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class Pusher {
    private final HardwareMap hardwareMap;
    public Servo Pusher;
    public static double Wait = 0.0;
    public static double Push = 0.3;
    public ElapsedTime LastShot = new ElapsedTime();
    public PushState state = PushState.RETURNING;
    public static double ShootTime = 0.5;
    public static double ReturnTime = 0.5;
    public enum PushState {
        WAITING,
        SHOOTING,
        RETURNING
    }

    public Pusher(HardwareMap hwm) {
        this.hardwareMap = hwm;
        Pusher = hardwareMap.get(Servo.class, "Pusher");

    }

    public void AttemptPush(ComponentShellTeleop Comps) {
        if (state == PushState.WAITING & Comps.shooter.state == Shooter.ShooterState.READY) {
            Pusher.setPosition(Push);
            state = PushState.SHOOTING;
            LastShot.reset();
        }
    }
    public void update(ComponentShellTeleop Comps) {
        switch (state) {
            case SHOOTING:
                if (LastShot.seconds() > ShootTime) {
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