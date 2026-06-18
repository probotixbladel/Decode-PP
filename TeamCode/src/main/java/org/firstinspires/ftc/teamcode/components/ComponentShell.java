package org.firstinspires.ftc.teamcode.components;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.bylazar.telemetry.TelemetryManager;

@Configurable
public class ComponentShell {
    public boolean takeOut = false;
    public boolean takeStatic = false;
    public boolean holdSpeed = false;
    public static Pose startingPoseBackBlue = new Pose(56, 9, Math.toRadians(270));
    public static Pose startingPoseBackRed = new Pose(87.0, 9, Math.toRadians(270));
    public static Pose startingPoseGoalBlue = new Pose(17, 121, Math.toRadians(143));
    public static Pose startingPoseGoalRed = new Pose(127, 121, Math.toRadians(37));
    public HardwareMap hardwareMap;
    public final Intake intake;
    public final Shooter shooter;
    public Pusher pusher;
    public Through through;
	public Floodgate floodgate;
    public Blinky blinky;
    public ArtifactDetector detector;
    public Follower follower;
    public TelemetryManager telemetryM;
    //public LimeLight limeLight;
    public Pose limePos = new Pose();
    public Alliance alliance;
    public boolean SinglePlayer;
    public boolean forcePush = false;
    public int shootNum;
    public static boolean useCam = false;
    public static double camCertanty = 0.10;

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
        //this.limeLight = new LimeLight(hardwareMap, alliance);
        this.blinky = new Blinky(hardwareMap);
        this.follower = flw;
        this.telemetryM = Tm;
        this.SinglePlayer = single;
        this.floodgate = new Floodgate(hardwareMap);
    }

	@SuppressLint("SuspiciousIndentation")
    public void update() {
		floodgate.update();
        detector.update(this);
        blinky.update(this);
        shooter.update(this);
        pusher.update(this);
        through.update(this);
        intake.update(this);

        /*
		if (useCam) {
            limePos = limeLight.update(telemetryM, Math.toDegrees(follower.getHeading()));
            if (limePos.getX() != 72 & limePos.getY() != 72) {
                follower.setX(limePos.getY() * camCertanty + follower.getPose().getX() * (1 - camCertanty));
                follower.setY(limePos.getX() * camCertanty + follower.getPose().getY() * (1 - camCertanty));
            }
		}
         */

        telemetryM.debug("aimpos", shooter.ShootTo);
		telemetryM.debug("alliance: ", alliance);
        telemetryM.debug("Pusher angle:", pusher.pusherAngle);
        telemetryM.debug("Pusher state:", pusher.state);
        // telemetryM.debug("detector dist", detector.distance);
        telemetryM.debug("Detecting:", detector.firstDetecting);

        telemetryM.debug("lime pos: ", limePos);
        telemetryM.debug("follower pos: ", follower.getPose());


        //telemetryM.debug("Number of shots left", shootNum);
        telemetryM.debug("intake:", intake.state);
        telemetryM.debug("Vel: ", shooter.CurrentVel, Shooter.TargetVel);
        telemetryM.debug("MinToMax: ", shooter.MinToMax);
        telemetryM.debug("shooter state: ", shooter.state);
        //telemetryM.debug("FloodgateCurrent", floodgate.floodgateCurrent);

    }

    public void updateTeleop(Gamepad gamepad1, Gamepad gamepad2) {
        this.update();
        if (SinglePlayer){
            //DEBUG
            if (gamepad2.dpadDownWasPressed()) {
                forcePush = !forcePush;
            }

            if(gamepad2.back && gamepad2.a) {
                follower.setPose(startingPoseBackBlue);
            }
            else if(gamepad2.back && gamepad2.b){
                follower.setPose(startingPoseBackRed);
            }
            else if(gamepad2.back && gamepad2.x){
                follower.setPose(startingPoseGoalBlue);
            }
            else if(gamepad2.back && gamepad2.y){
                follower.setPose(startingPoseGoalRed);
            }
            else if(gamepad2.back && gamepad2.dpad_up){
                follower.setHeading(-90);
            }

            holdSpeed = gamepad2.left_trigger_pressed && gamepad2.right_trigger_pressed;


            //CONTROLS
            if (gamepad1.left_trigger_pressed) {
                intake.state = Intake.IntakeState.OUTTAKE;
                through.state = Through.ThroughState.IN_THROUGH;
            } else if (gamepad1.right_trigger_pressed) {
                if (forcePush) {
                    pusher.forcePush();
                } else {
                    pusher.AttemptPush(this);
                }
                through.state = Through.ThroughState.IN_THROUGH;
                intake.state = Intake.IntakeState.INTAKE;
            } else {
                through.state = Through.ThroughState.OFF;
                intake.state = Intake.IntakeState.INTAKE;
            }

        } else {
            //DEBUG
            if (gamepad2.dpadDownWasPressed()) {
                forcePush = !forcePush;
            }

            if(gamepad2.back && gamepad2.a) {
                follower.setPose(startingPoseBackBlue);
            }
            else if(gamepad2.back && gamepad2.b){
                follower.setPose(startingPoseBackRed);
            }
            else if(gamepad2.back && gamepad2.x){
                follower.setPose(startingPoseGoalBlue);
            }
            else if(gamepad2.back && gamepad2.y){
                follower.setPose(startingPoseGoalRed);
            }
            else if(gamepad2.back && gamepad2.dpad_up){
                follower.setHeading(-90);
            }

            holdSpeed = gamepad2.left_trigger_pressed && gamepad2.right_trigger_pressed;


            //CONTROLS
            if (gamepad2.left_trigger_pressed) {
                intake.state = Intake.IntakeState.OUTTAKE;
                through.state = Through.ThroughState.IN_THROUGH;
            } else if (gamepad1.right_trigger_pressed) {
                if (forcePush) {
                    pusher.forcePush();
                } else {
                    pusher.AttemptPush(this);
                }
                through.state = Through.ThroughState.IN_THROUGH;
                intake.state = Intake.IntakeState.INTAKE;
            } else {
                through.state = Through.ThroughState.OFF;
                intake.state = Intake.IntakeState.INTAKE;
            }


        }
    }

    public void ResetShootNum(){
		shootNum = 0;
	}

    public void AutoShooterStart(){
        through.state = Through.ThroughState.IN_THROUGH;
        if(pusher.AttemptPush(this)) {
            shootNum += 1;
        }
    }

    public boolean FinishedShooting(int num){
        if(shootNum >= num){
            through.state = Through.ThroughState.OFF;
            return true;
        }
        return false;
    }


}
