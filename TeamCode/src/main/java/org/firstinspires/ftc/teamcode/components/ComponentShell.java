package org.firstinspires.ftc.teamcode.components;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;


public class ComponentShell {
    private final HardwareMap hardwareMap;
    private final Intake intake;
    private Follower follower;

    public ComponentShell(HardwareMap hwm, Follower flw) {
        this.hardwareMap = hwm;
        this.intake = new Intake(hardwareMap);
        this.follower = flw;

    }

    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        if      (gamepad2.right_trigger > 0.2) {intake.state = Intake.IntakeState.INTAKE; }
        else if (gamepad2.left_trigger  > 0.2) {intake.state = Intake.IntakeState.OUTTAKE;}
        else                                   {intake.state = Intake.IntakeState.OFF;    }


        intake.update();
    }


}
