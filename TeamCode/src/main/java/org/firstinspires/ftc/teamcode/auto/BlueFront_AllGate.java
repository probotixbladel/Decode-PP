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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.ComponentShell;
import org.firstinspires.ftc.teamcode.components.Pusher;
import org.firstinspires.ftc.teamcode.components.Storage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Configurable
@Autonomous(name = "BlueFront_AllGate")
public class BlueFront_AllGate extends OpMode {
    private Follower follower;
    public ElapsedTime Timer = new ElapsedTime();
    private Timer pathTimer, actionTimer, opmodeTimer;
    public static double gateTime = 2;
    private int pathState;
    private final Pose startPose = new Pose(17.4, 120.3, Math.toRadians(-36)); // Starting pose for our robot
    private final Pose scorePreloadSetupPose = new Pose(37.1, 107.0, Math.toRadians(-44)); // Pose for setting up scoring preload (with SOTM)

    private final Pose scorePreloadFinalPose = new Pose(57.8, 86, Math.toRadians(-46)); // Final pose for scoring preload (with SOTM)
    private final Pose pickup2Pose = new Pose(17.8, 59.9, Math.toRadians(180)); // Pose for picking up second row of artifacts
    private final Pose pickup2ControlPoint = new Pose(81.7, 60); // Control point for pickup 2 for an arc path
    private final Pose scorePose = new Pose(47.4, 95.5, Math.toRadians(-46)); // Score pose
    private final Pose scoreControlPoint = new Pose(59.6, 73); // Control point for score pose for an arc path
    private final Pose gatePickupPose = new Pose(11.4, 60.2, Math.toRadians(148)); // Pose for pickup up artifacts from the gate
    private final Pose gatePickupControlPoint = new Pose(62.4, 53.7); // Control point for gate pickup
    private final Pose scorePosePreload = new Pose(37.1, 107.0, Math.toRadians(-44));
    private final Pose pickup1Setup = new Pose(47, 84, Math.toRadians(180));
    private final Pose pickup1Pose = new Pose(20, 84, Math.toRadians(180));
    private final Pose gateSetup = new Pose(25, 75, Math.toRadians(180));
    private final Pose gateOpen = new Pose(14, 75, Math.toRadians(180));
    private final Pose scorePose1 = new Pose(54, 90, Math.toRadians(-45));
    private final Pose pickup2Setup = new Pose(48, 60, Math.toRadians(180));
    //private final Pose pickup2Pose = new Pose(17, 60, Math.toRadians(180));
    private final Pose scorePose2 = new Pose(48, 90, Math.toRadians(-48));
    private final Pose pickup3Setup = new Pose(42, 38, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(17, 38, Math.toRadians(180));
    private final Pose scorePose3 = new Pose(55, 89, Math.toRadians(-46));
    private final Pose leaveTrianglePose = new Pose(50,60, Math.toRadians(-46));
    public Path scorePreload;
    public ComponentShell comps;
    public PathChain grabPickup1, openGate, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, grabPickupSetup1, grabPickupSetup2, grabPickupSetup3, leave;
    public int Shots = 0;
    private TelemetryManager telemetryM;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePosePreload));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePosePreload.getHeading());


        grabPickupSetup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePosePreload, pickup1Setup))
                .setLinearHeadingInterpolation(scorePosePreload.getHeading(), pickup1Pose.getHeading())
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
                .addPath(new BezierLine(gateOpen, scorePose1))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose1.getHeading())
                .build();

        grabPickupSetup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, pickup2Setup))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickup2Pose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Setup, pickup2Pose))
                .setLinearHeadingInterpolation(pickup2Setup.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose2))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose2.getHeading())
                .build();

        grabPickupSetup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, pickup3Setup))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup3Pose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Setup, pickup3Pose))
                .setLinearHeadingInterpolation(pickup3Setup.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose3))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose3.getHeading())
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose3, leaveTrianglePose))
                .setLinearHeadingInterpolation(scorePose3.getHeading(), leaveTrianglePose.getHeading())
                .build();
    }


    public void autonomousPathUpdate() {

        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                comps.through.InThrough();
                comps.shooter.PreTargetTo(scorePosePreload);
                setPathState(1);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    comps.ResetShootNum();
                    comps.shooter.Arrived();
                    setPathState(2);
                }
                break;

            case 2:
                comps.AutoShooterStart();
                if(comps.FinishedShooting(3) && (comps.pusher.state == Pusher.PushState.WAITING || comps.pusher.state == Pusher.PushState.RELOADING))
                {
                    setPathState(3);
                    comps.shooter.PreTargetTo(scorePose1);
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
                    comps.intake.TakeIn(comps);
                    follower.followPath(grabPickup1, 1, true);
                    setPathState(5);
                }
                break;

            case 5:
                if(!follower.isBusy()){
                    comps.intake.StaticIntake();
                    follower.followPath(openGate,true);
                    setPathState(6);
                }
                break;

            case 6:
                Timer.reset();
                setPathState(7);
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
                    comps.shooter.Arrived();
                    setPathState(9);
                }
                break;

            case 9:
                comps.AutoShooterStart();
                if(comps.FinishedShooting(3) && (comps.pusher.state == Pusher.PushState.WAITING || comps.pusher.state == Pusher.PushState.RELOADING)){
                    comps.shooter.PreTargetTo(scorePose2);
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
                    comps.intake.TakeIn(comps);
                    follower.followPath(grabPickup2, 1, true);
                    setPathState(12);
                }
                break;

            case 12:
                if(!follower.isBusy()) {
                    comps.intake.StaticIntake();
                    follower.followPath(scorePickup2, true);
                    setPathState(13);
                }
                break;

            case 13:
                if(!follower.isBusy()){
                    comps.ResetShootNum();
                    comps.shooter.Arrived();
                    setPathState(14);
                }
                break;

            case 14:
                comps.AutoShooterStart();
                if(comps.FinishedShooting(3) && (comps.pusher.state == Pusher.PushState.WAITING || comps.pusher.state == Pusher.PushState.RELOADING)){
                    comps.shooter.PreTargetTo(scorePose3);
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
                    comps.intake.TakeIn(comps);
                    follower.followPath(grabPickup3, 1, true);
                    setPathState(17);
                }
                break;

            case 17:
                if(!follower.isBusy()) {
                    comps.intake.StaticIntake();
                    follower.followPath(scorePickup3, true);
                    setPathState(18);
                }
                break;
            case 18:
                if(!follower.isBusy()){
                    comps.ResetShootNum();
                    comps.shooter.Arrived();
                    setPathState(19);
                }
                break;

            case 19:
                comps.AutoShooterStart();
                if(comps.FinishedShooting(3) && (comps.pusher.state == Pusher.PushState.WAITING || comps.pusher.state == Pusher.PushState.RELOADING)){
                    setPathState(20);
                }
                break;

            case 20:
                comps.intake.StaticIntake();
                comps.through.StaticThrough();
                follower.followPath(leave, true);
                setPathState(-1);
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