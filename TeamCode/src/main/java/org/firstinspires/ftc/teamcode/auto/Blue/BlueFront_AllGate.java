package org.firstinspires.ftc.teamcode.auto.Blue;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.ComponentShell;
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
    private final Pose startPose = new Pose(17.4, 120.4, Math.toRadians(-36));
    private final Pose scorePoseStart = new Pose(37.1, 107.0, Math.toRadians(-44));
    private final Pose scorePoseFinal = new Pose(57.8, 86.0, Math.toRadians(-46));
    private final Pose pickup2Pose = new Pose(17.8, 59.9, Math.toRadians(180));
    private final Pose pickup2Controlpoint = new Pose(81.7, 56.1);
    private final Pose scorePose = new Pose(47.4, 95.5, Math.toRadians(-46));
    private final Pose scoreControlpoint = new Pose(59.6, 73.0);
    private final Pose pickupGatePose = new Pose(15.4, 60.2, Math.toRadians(148));
    private final Pose pickupGateControlpoint = new Pose(62.4, 53.7);
    private final Pose pickup1Pose = new Pose(17.1, 83.6, Math.toRadians(180));
    private final Pose pickup1Controlpoint = new Pose(69.5, 86.7);
    private final Pose leavePose = new Pose(58.3, 53.0, Math.toRadians(-50));
    private final Pose leaveControlpose = new Pose(77.1, 79.1);
    public ComponentShell comps;
    public PathChain scorePreloadStart, scorePreloadFinal, grabGate, scoreGate, grabPickup1, grabPickup2, scorePickup1, scorePickup2, leave;
    public int Shots = 0;
    private TelemetryManager telemetryM;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        //scorePreload = new Path(new BezierLine(startPose, scorePosePreload));
        //scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePosePreload.getHeading());
        scorePreloadStart = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePoseStart))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePoseStart.getHeading())
                .build();

        scorePreloadFinal = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseStart, scorePoseFinal))
                .setLinearHeadingInterpolation(scorePoseStart.getHeading(), scorePoseFinal.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePoseFinal, pickup2Controlpoint, pickup2Pose))
                .setLinearHeadingInterpolation(scorePoseFinal.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, scoreControlpoint, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabGate = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickupGateControlpoint, pickupGatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupGatePose.getHeading())
                .build();

        scoreGate = follower.pathBuilder()
                .addPath(new BezierCurve(pickupGatePose, scoreControlpoint, scorePose))
                .setLinearHeadingInterpolation(pickupGatePose.getHeading(), pickupGatePose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup1Controlpoint, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, leaveControlpose, leavePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leavePose.getHeading())
                .build();
    }


    public void autonomousPathUpdate() {

        switch (pathState) {
            case 0:
                follower.followPath(scorePreloadStart);
                comps.through.InThrough();
                comps.shooter.PreTargetTo(scorePoseStart);
                nextPathState();
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    follower.followPath(scorePreloadFinal);
                    nextPathState();
                }
                break;

            case 2:
                if(!follower.isBusy()){
                    follower.followPath(grabPickup2);
                    nextPathState();
                }
                break;

            case 3:
                if(!follower.isBusy()){
                    follower.followPath(scorePickup2,true);
                    nextPathState();
                }
                break;

            case 4:
                if(!follower.isBusy()){
                    comps.intake.TakeIn(comps);
                    follower.followPath(grabGate, 1, true);
                    nextPathState();
                }
                break;

            case 5:
                if(!follower.isBusy()){
                    comps.intake.StaticIntake();
                    follower.followPath(scoreGate,true);
                    nextPathState();
                }
                break;

            case 6:
                if(!follower.isBusy()){
                    follower.followPath(grabPickup1, true);
                    nextPathState();
                }
                break;

            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    nextPathState();
                }
                break;

            case 8:
                if(!follower.isBusy()){
                    follower.followPath(leave);
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

    public void nextPathState(){
        pathState += 1;
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