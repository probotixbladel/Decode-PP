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
    public static double intake_power = 1.0;
    public static double outtake_power = -0.6;
    public static double static_power = 0.3;
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

    public void TakeIn() {
        state = IntakeState.INTAKE;
        intake.setPower(intake_power);
    }
    public void TakeOut() {
        state = Intake.IntakeState.OUTTAKE;
        intake.setPower(outtake_power);
    }
    public void StaticIntake() {
        state = Intake.IntakeState.OFF;
        intake.setPower(static_power);
    }


}