package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
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
    public static double RestAngle = 340;
    public static double AriveAngle = 280;
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

    public boolean AttemptPush(ComponentShell Comps) {
        //if (!Comps.shooter.PreTargeting & Comps.follower.getAngularVelocity() < 0.314 & Comps.follower.getVelocity().getMagnitude() < 5) {
            if (state == PushState.WAITING & Comps.shooter.state == Shooter.ShooterState.READY) {
                Pusher.setPosition(Push);
                state = PushState.SHOOTING;
                LastShot.reset();
                return true;
            }
        //}
        return false;
    }
    public void update(ComponentShell Comps) {
        PusherAngle = PusherEnc.getVoltage() / 3.3 * 360;
        switch (state) {
            case SHOOTING:
                if (LastShot.seconds() > ShootTime || (PusherAngle < AriveAngle && PusherAngle > 20)) {
                    Pusher.setPosition(Wait);
                    state = PushState.RETURNING;
                    LastShot.reset();
                }
            case RETURNING:
                if (LastShot.seconds() > ReturnTime) {
                    state = PushState.WAITING;
                }
        }
    }
}