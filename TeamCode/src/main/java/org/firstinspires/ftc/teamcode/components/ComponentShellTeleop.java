package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;
import com.bylazar.telemetry.TelemetryManager;

@Configurable
public class ComponentShellTeleop {
    public HardwareMap hardwareMap;
    public final Intake intake;
    public final Shooter shooter;
    public Pusher pusher;
    public Through through;
    public Follower follower;
    public TelemetryManager telemetryM;
    public boolean RunningAuto;


    public ComponentShellTeleop(HardwareMap hwm, Follower flw, TelemetryManager Tm) {
        this.hardwareMap = hwm;
        this.intake = new Intake(hardwareMap);
        this.shooter = new Shooter(hardwareMap);
        this.pusher = new Pusher(hardwareMap);
        this.through = new Through(hardwareMap);
        this.follower = flw;
        this.telemetryM = Tm;
    }

    public void update() {
        pusher.update(this);
        shooter.update();
    }
    public void updateTeleop(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad2.a) {
            pusher.AttemptPush(this);
        }
        if (gamepad2.right_trigger > 0.2) {
            intake.TakeIn();
        }
        else if (gamepad2.left_trigger > 0.2) {
            intake.TakeOut();
        }
        else {
            intake.StaticIntake();
        }

        if(gamepad2.b) {
            shooter.ChangeShooterSpeed();
        }

        if (gamepad2.x) {
            through.InThrough(this);
        }
        else if (gamepad2.left_bumper) {
            through.OutThrough(this);
        }
        else {
            through.StaticThrough(this);
        }

        this.update();

        telemetryM.debug("Vel: ", shooter.CurrentVel);
        telemetryM.debug("shooter state: ", shooter.state);
        telemetryM.debug("intake state: ", intake.state);
        telemetryM.debug("power", intake.intake.getPower(), through.Through.getPower());
    }
}
