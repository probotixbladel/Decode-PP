package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
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
    public Pose limePos = new Pose();
    public Alliance alliance;
    public boolean SinglePlayer;
    public int shootNum;
    public Floodgate Floodgate;

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
        this.Floodgate = new Floodgate(hardwareMap);
    }

    public void update() {
        Pose pos = limeLight.update(this, telemetryM, Math.toDegrees(follower.getHeading()));
        shooter.update();
        pusher.update(this);
        shooter.setSpeeds(follower.getPose());
        detector.update();
        through.update(this);
        Floodgate.update(this);

        if (pos != null) {
            limePos = pos;
        }

        telemetryM.debug("alliance: ", alliance);
        telemetryM.debug("Pusher angle:", pusher.PusherAngle);
        telemetryM.debug("Pusher state:", pusher.state);
        telemetryM.debug("detector dist", detector.distance);
        telemetryM.debug("lime pos: ", limePos);
        telemetryM.debug("Number of shots left", shootNum);
        telemetryM.debug("follower pos: ", follower.getPose());
        telemetryM.debug("through state: ", through.state);
        telemetryM.debug("Vel: ", shooter.CurrentVel, shooter.TargetVel, "dist", shooter.setSpeeds(follower.getPose()));
        telemetryM.debug("shooter state: ", shooter.state);
        telemetryM.debug("FloodgateCurrent", Floodgate.FloodgateCurrent);

    }

    public void updateTeleop(Gamepad gamepad1, Gamepad gamepad2) {
        this.update();
        if (SinglePlayer){
            if (gamepad1.a) {
                pusher.AttemptPush(this);
            }
            if (gamepad1.right_trigger > 0.2) {
                intake.TakeIn(this);
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
                intake.TakeIn(this);
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
    public void ResetShootNum(){shootNum = 0;}
    public void AutoShooterStart(){
        if(pusher.AttemptPush(this)){
            shootNum += 1;
        }
        intake.TakeIn(this);
    }
    public boolean FinishedShooting(int num){
        if(shootNum >= num){
            intake.StaticIntake();
            return true;
        }
        return false;
    }


}

