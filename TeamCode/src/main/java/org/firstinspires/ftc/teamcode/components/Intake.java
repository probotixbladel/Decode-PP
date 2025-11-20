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
    public DcMotorEx Through;
    private static double intake_power = 1.0;
    private static double outtake_power = -0.6;
    private static double static_power = 0.3;
    public IntakeState state = IntakeState.OFF;
    public IntakeState last_state;
    public boolean swith = false;
    public enum IntakeState {
        INTAKE,
        OUTTAKE,
        OFF
    }
    public Intake(HardwareMap hwm) {
        this.hardwareMap = hwm;
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        Through = hardwareMap.get(DcMotorEx.class, "Through");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update(Gamepad gamepad2) {
        if (gamepad2.right_trigger > 0.2) {
            intake.setPower(intake_power);
            Through.setPower(intake_power);
        }
        else if (gamepad2.left_trigger  > 0.2) {
            state = Intake.IntakeState.OUTTAKE;
            intake.setPower(outtake_power);
            Through.setPower(outtake_power);
        }
        else {
            state = Intake.IntakeState.OFF;
            intake.setPower(static_power);
            Through.setPower(static_power);
        }
        swith = state != last_state;
        /*
        if (swith) {
            switch (state) {
                case INTAKE:

                case OUTTAKE:

                case OFF:

            }
        }
        */
        last_state = state;

    }

}