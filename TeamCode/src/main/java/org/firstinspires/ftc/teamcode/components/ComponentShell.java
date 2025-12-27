package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;
import com.bylazar.telemetry.TelemetryManager;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Configurable
public class ComponentShell {
    public HardwareMap hardwareMap;
    public final Intake intake;
    public final Shooter shooter;
    public Pusher pusher;
    public Through through;
    public ArtifactDetector detector;
    public Follower follower;
    public TelemetryManager telemetryM;
    public LimeLight limeLight;
    public Pose3D limePos = new Pose3D(new Position(), new YawPitchRollAngles(AngleUnit.DEGREES,0,0,0 ,0) );
    public Alliance alliance;
    public boolean RunningAuto;
    public boolean SinglePlayer;
    public enum Alliance {
        BLUE,
        RED
    }


    public ComponentShell(HardwareMap hwm, Follower flw, TelemetryManager Tm, Alliance al, boolean single) {
        this.alliance = al;
        this.hardwareMap = hwm;
        this.detector = new ArtifactDetector(hardwareMap);
        this.intake = new Intake(hardwareMap);
        this.shooter = new Shooter(hardwareMap, alliance);
        this.pusher = new Pusher(hardwareMap);
        this.through = new Through(hardwareMap);
        this.limeLight = new LimeLight(hardwareMap, alliance);
        this.follower = flw;
        this.telemetryM = Tm;
        this.SinglePlayer = single;
    }

    public void update() {
        Pose3D pos = limeLight.update(this, telemetryM, follower.getHeading());
        shooter.update();
        pusher.update(this);
        shooter.setSpeeds(follower.getPose());
        detector.update();

        if (pos != null) {
            limePos = pos;
        }
        telemetryM.debug("detector dist", detector.distance);
        telemetryM.debug("lime pos: ", limePos);
        telemetryM.debug("folower pos: ", follower.getPose());
        telemetryM.debug("Pusher angle:", pusher.PusherAngle);
        telemetryM.debug("Vel: ", shooter.CurrentVel);
        telemetryM.debug("shooter state: ", shooter.state);
    }
    public void updateTeleop(Gamepad gamepad1, Gamepad gamepad2) {
        this.update();
        if (!SinglePlayer){
            if (gamepad1.a) {
                pusher.AttemptPush(this);
            }
            if (gamepad1.right_trigger > 0.2) {
                intake.TakeIn();
            } else if (gamepad1.left_trigger > 0.2) {
                intake.TakeOut();
            } else {
                intake.StaticIntake();
            }

            if (gamepad1.x) {
                through.InThrough(this);
            } else if (gamepad1.left_bumper) {
                through.OutThrough(this);
            } else {
                through.StaticThrough(this);
            }

        } else {
            if (gamepad2.a) {
                pusher.AttemptPush(this);
            }
            if (gamepad2.right_trigger > 0.2) {
                intake.TakeIn();
            } else if (gamepad2.left_trigger > 0.2) {
                intake.TakeOut();
            } else {
                intake.StaticIntake();
            }

            if (gamepad2.x) {
                through.InThrough(this);
            } else if (gamepad2.left_bumper) {
                through.OutThrough(this);
            } else {
                through.StaticThrough(this);
            }
        }


    }
}
