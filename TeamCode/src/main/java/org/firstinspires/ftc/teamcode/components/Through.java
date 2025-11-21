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
    private static double in_power = 1.0;
    private static double out_power = -1;
    private static double static_power = 0;

    public Through(HardwareMap hwm) {
        this.hardwareMap = hwm;
        Through = hardwareMap.get(DcMotorEx.class, "Through");
        Through.setDirection(DcMotor.Direction.REVERSE);
        Through.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update(Gamepad gamepad2, ComponentShell Comps) {
        if (Comps.intake.state == Intake.IntakeState.OUTTAKE) {
            Through.setPower(in_power);
        }
        else if (Comps.pusher.state == Pusher.PushState.WAITING) {
            if(gamepad2.x)
            {
                Through.setPower(out_power);
            }
        else {
            Through.setPower(static_power);
        }
    }

}}