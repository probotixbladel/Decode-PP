package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Pusher {
    private HardwareMap hardwareMap;
    public Servo Pusher;
    public static double Wait = 0.5;
    public static double Push = 0.8;
    public boolean Pushing = false;

    public Pusher(HardwareMap hwm) {
        this.hardwareMap = hwm;
        Pusher = hardwareMap.get(Servo.class, "Pusher");

    }

    public void update(Gamepad gamepad2, ComponentShell Comps) {
        if (gamepad2.aWasPressed() & Comps.shooter.state == Shooter.ShooterState.READY){
            Pusher.setPosition(Push);
            Pushing = true;

        }

    }

}