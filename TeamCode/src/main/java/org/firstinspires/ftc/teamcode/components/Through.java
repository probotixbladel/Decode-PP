package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Through {
    private HardwareMap hardwareMap;
    public DcMotorEx Through;
    public static double in_power = 1.0;
    public static double out_power = -1;
    public static double static_power = 0;
    public static double loosen_power = -0.2;
    public ThroughState state = ThroughState.OFF;
    public enum ThroughState {
        INTHROUGH,
        OUTTHROUGH,
        OFF
    }

    public Through(HardwareMap hwm) {
        this.hardwareMap = hwm;
        Through = hardwareMap.get(DcMotorEx.class, "Through");
        Through.setDirection(DcMotor.Direction.FORWARD);
        Through.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void OutThrough(ComponentShell Comps) {
        state = ThroughState.OUTTHROUGH;
    }
    public void StaticThrough(ComponentShell Comps) {
        state = ThroughState.OFF;
    }
    public void update(ComponentShell Comps) {
        switch (state) {
            case INTHROUGH:
                if (Comps.pusher.state == Pusher.PushState.WAITING || Comps.pusher.state == Pusher.PushState.RELOADING) {
                    Through.setPower(in_power);
                }
                else {
                    Through.setPower(loosen_power);
                }
                break;
            case OUTTHROUGH:
                Through.setPower(out_power);
                break;
            case OFF:
                Through.setPower(static_power);
                break;
        }
    }

    public void InThrough(ComponentShell Comps) {
        state = ThroughState.INTHROUGH;
    }
}