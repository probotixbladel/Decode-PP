package org.firstinspires.ftc.teamcode.components;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final HardwareMap hardwareMap;
    public DcMotorEx intake;
    private static final double intake_power = 1.0;
    private static final double outtake_power = -0.4;
    private static final double static_power = 0.1;
    public IntakeState state = IntakeState.OFF;
    public enum IntakeState {
        INTAKE,
        OUTTAKE,
        OFF
    }
    public Intake(HardwareMap hwm) {
        this.hardwareMap = hwm;
        intake = hardwareMap.get(DcMotorEx.class, "intake");
    }

    public void update(){
        switch (state) {
            case INTAKE:
                intake.setPower(intake_power);
            case OUTTAKE:
                intake.setPower(outtake_power);
            case OFF:
                intake.setPower(static_power);
        }
    }

}