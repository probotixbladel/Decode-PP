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

    public Through(HardwareMap hwm) {
        this.hardwareMap = hwm;
        Through = hardwareMap.get(DcMotorEx.class, "Through");
        Through.setDirection(DcMotor.Direction.FORWARD);
        Through.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void OutThrough(ComponentShell Comps) {
        Through.setPower(out_power);
    }
    public void InThrough(ComponentShell Comps) {
        if (Comps.pusher.state == Pusher.PushState.WAITING) {
            Through.setPower(in_power);
        } else {
            Through.setPower(-0.1);
        }
    }
    public void StaticThrough(ComponentShell Comps) {
        if (Comps.pusher.state == Pusher.PushState.WAITING) {
            Through.setPower(static_power);
        }
    }
}