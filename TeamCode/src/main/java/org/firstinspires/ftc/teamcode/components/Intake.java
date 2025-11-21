package org.firstinspires.ftc.teamcode.components;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Intake {
    private HardwareMap hardwareMap;
    public DcMotorEx intake;
    private static double intake_power = 1.0;
    private static double outtake_power = -0.6;
    private static double static_power = 0.3;
    public IntakeState state = IntakeState.OFF;
    public enum IntakeState {
        INTAKE,
        OUTTAKE,
        OFF
    }
    public Intake(HardwareMap hwm) {
        this.hardwareMap = hwm;
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update(Gamepad gamepad2) {
        if (gamepad2.right_trigger > 0.2) {
            state = IntakeState.INTAKE;
            intake.setPower(intake_power);
        }
        else if (gamepad2.left_trigger  > 0.2) {
            state = Intake.IntakeState.OUTTAKE;
            intake.setPower(outtake_power);
        }
        else {
            state = Intake.IntakeState.OFF;
            intake.setPower(static_power);
        }
    }

}