package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.ComponentShell;
import org.firstinspires.ftc.teamcode.components.Pusher;
import org.firstinspires.ftc.teamcode.components.Storage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "BlueGoal", group = "Examples")
public class BlueGoal extends OpMode {
    private Follower follower;
    public ElapsedTime Timer = new ElapsedTime();
    private Timer pathTimer, actionTimer, opmodeTimer;
    public static double gateTime = 1;
    private int pathState;
    private final Pose startPose = new Pose(25, 131, Math.toRadians(-36)); //See ExampleAuto to understand how to use this
    private final Pose scorePose = new Pose(59, 85, Math.toRadians(-48)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Setup = new Pose(50, 85, Math.toRadians(180)); // Setup to pickup the highest set of balls
    private final Pose pickup1Pose = new Pose(21, 85, Math.toRadians(180));// Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose gateSetup = new Pose(21, 79, Math.toRadians(180)); // Stand infront of the gate
    private final Pose gateOpen = new Pose(18, 79, Math.toRadians(180)); // OPEN DA GATEHHH
    private final Pose pickup2Setup = new Pose(50, 62, Math.toRadians(180)); // Setup to pickup the middle set of balls
    private final Pose pickup2Pose = new Pose(18, 62, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Setup = new Pose(50, 38, Math.toRadians(180)); // Setup to pickup the lowest set of balls
    private final Pose pickup3Pose = new Pose(18, 38, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    public Path scorePreload;
    public ComponentShell comps;
    public PathChain grabPickup1, openGate, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, grabPickupSetup1, grabPickupSetup2, grabPickupSetup3;
    public int Shots = 0;
    private TelemetryManager telemetryM;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());


        grabPickupSetup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Setup))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Setup, pickup1Pose))
                .setLinearHeadingInterpolation(pickup1Setup.getHeading(), pickup1Pose.getHeading())
                .build();

        openGate = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, gateSetup))
                .setLinearHeadingInterpolation(pickup1Setup.getHeading(), pickup1Pose.getHeading())
                .addPath(new BezierLine(gateSetup, gateOpen))
                .setLinearHeadingInterpolation(pickup1Setup.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(gateOpen, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickupSetup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Setup))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Setup, pickup2Pose))
                .setLinearHeadingInterpolation(pickup2Setup.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickupSetup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Setup))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Setup, pickup3Pose))
                .setLinearHeadingInterpolation(pickup3Setup.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }


    public void autonomousPathUpdate() {

        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                comps.shooter.PreTargetTo(scorePose);
                setPathState(1);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    comps.ResetShootNum();
                    comps.shooter.Arived();
                    setPathState(2);
                }
                break;

            case 2:
                comps.AutoShooterStart();
                if(comps.FinishedShooting(3) && comps.pusher.state == Pusher.PushState.WAITING)
                {
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()){
                    follower.followPath(grabPickupSetup1,true);
                    setPathState(4);
                }
                break;

            case 4:
                if(!follower.isBusy()){
                    comps.intake.TakeIn();
                    comps.through.InThrough(comps);
                    follower.followPath(grabPickup1, 0.7, true);
                    setPathState(5);
                }
                break;

            case 5:
                if(!follower.isBusy()){
                    comps.intake.StaticIntake();
                    comps.through.StaticThrough(comps);
                    follower.followPath(openGate, 0.5, true);
                    setPathState(6);
                }
                break;

            case 6:
                if(!follower.isBusy()){
                    Timer.reset();
                    setPathState(7);
                }
                break;

            case 7:
                if(Timer.seconds() > gateTime) {
                    follower.followPath(scorePickup1, true);
                    setPathState(8);
                }
                break;

            case 8:
                if(!follower.isBusy()){
                    comps.ResetShootNum();
                    comps.shooter.Arived();
                    setPathState(9);
                }
                break;

            case 9:
                comps.AutoShooterStart();
                if(comps.FinishedShooting(3) && comps.pusher.state == Pusher.PushState.WAITING){
                    setPathState(10);
                }
                break;

            case 10:
                if(!follower.isBusy()){
                    follower.followPath(grabPickupSetup2,true);
                    setPathState(11);
                }
                break;

            case 11:
                if(!follower.isBusy()){
                    comps.intake.TakeIn();
                    comps.through.InThrough(comps);
                    follower.followPath(grabPickup2, 0.7, true);
                    setPathState(12);
                }
                break;

            case 12:
                if(!follower.isBusy()) {
                    comps.intake.StaticIntake();
                    comps.through.StaticThrough(comps);
                    follower.followPath(scorePickup2, true);
                    setPathState(13);
                }
                break;

            case 13:
                if(!follower.isBusy()){
                    comps.ResetShootNum();
                    comps.shooter.Arived();
                    setPathState(14);
                }
                break;

            case 14:
                comps.AutoShooterStart();
                if(comps.FinishedShooting(3) && comps.pusher.state == Pusher.PushState.WAITING){
                    setPathState(15);
                }
                break;

            case 15:
                if(!follower.isBusy()){
                    follower.followPath(grabPickupSetup3,true);
                    setPathState(16);
                }
                break;

            case 16:
                if(!follower.isBusy()){
                    comps.intake.TakeIn();
                    comps.through.InThrough(comps);
                    follower.followPath(grabPickup3, 0.7, true);
                    setPathState(17);
                }
                break;

            case 17:
                if(!follower.isBusy()) {
                    comps.intake.StaticIntake();
                    comps.through.StaticThrough(comps);
                    follower.followPath(scorePickup3, true);
                    setPathState(18);
                }
                break;
            case 18:
                if(!follower.isBusy()){
                    comps.ResetShootNum();
                    comps.shooter.Arived();
                    setPathState(19);
                }
                break;

            case 19:
                comps.AutoShooterStart();
                if(comps.FinishedShooting(3) && comps.pusher.state == Pusher.PushState.WAITING){
                    setPathState(-1);
                }
                break;

        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        comps = new ComponentShell(hardwareMap, follower, telemetryM, ComponentShell.Alliance.BLUE, true);
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetryM.debug("Shots", Shots);

        comps.update();

        telemetryM.debug("path state", pathState);
        telemetryM.debug("x", follower.getPose().getX());
        telemetryM.debug("y", follower.getPose().getY());
        telemetryM.debug("heading", follower.getPose().getHeading());
        telemetryM.debug("Timer: ", Timer.seconds());
        telemetryM.debug("isBusy: ", follower.isBusy());
        telemetryM.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
        Storage.write(ComponentShell.Alliance.BLUE, follower.getPose());
    }
}
