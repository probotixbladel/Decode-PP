package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;
import com.bylazar.telemetry.TelemetryManager;

@Configurable
public class ComponentShell {
    public HardwareMap hardwareMap;
    public final Intake intake;
    public final Shooter shooter;
    public Pusher pusher;
    public Through through;
    public Follower follower;
    public TelemetryManager telemetryM;


    public ComponentShell(HardwareMap hwm, Follower flw, TelemetryManager Tm) {
        this.hardwareMap = hwm;
        this.intake = new Intake(hardwareMap);
        this.shooter = new Shooter(hardwareMap);
        this.pusher = new Pusher(hardwareMap);
        this.through = new Through(hardwareMap);
        this.follower = flw;
        this.telemetryM = Tm;

    }

    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        intake.update(gamepad2);
        shooter.update(gamepad2);
        pusher.update(gamepad2, this);
        through.update(gamepad2, this);

        telemetryM.debug("Vel: ", shooter.CurrentVel);
        telemetryM.debug("shooter state: ", shooter.state);
        telemetryM.debug("intake state: ", intake.state);
        telemetryM.debug("power", intake.intake.getPower(), through.Through.getPower());
    }
}
