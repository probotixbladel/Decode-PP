package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

@Configurable
public class ComponentShell {
    private HardwareMap hardwareMap;
    private final Intake intake;
    private final Shooter shooter;
    private Follower follower;
    private TelemetryManager telemetryM;


    public ComponentShell(HardwareMap hwm, Follower flw, TelemetryManager Tm) {
        this.hardwareMap = hwm;
        this.intake = new Intake(hardwareMap);
        this.shooter = new Shooter(hardwareMap);
        this.follower = flw;
        this.telemetryM = Tm;

    }

    public void update(Gamepad gamepad1, Gamepad gamepad2) {



        intake.update(gamepad2);
        shooter.update();

        telemetryM.debug("Vel: ", shooter.CurrentVel);
        telemetryM.debug("shooter state: ", shooter.state);
        telemetryM.debug("intake state: ", intake.state);
        telemetryM.debug("power", intake.intake.getPower(), intake.Through.getPower());
        telemetryM.debug("swith", intake.swith);

    }


}
