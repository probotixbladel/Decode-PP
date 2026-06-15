package org.firstinspires.ftc.teamcode.auto.Red;

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
@Autonomous(name = "RedBack_AllHp_Without3")
public class RedBack_AllHp_Without3 extends OpMode {
    private Follower follower;
    public ElapsedTime Timer = new ElapsedTime();
    private Timer pathTimer, actionTimer, opmodeTimer;
    public static double gateTime = 2;
    private int pathState;
    private final Pose startPose = new Pose(88.55, 8.28, Math.toRadians(90));
    private final Pose scorePose = new Pose(84.35, 16.65, Math.toRadians(-111));
    private final Pose pickupHpPose = new Pose(123.27, 36.36, Math.toRadians(0));
    private final Pose pickupHpControlpoint = new Pose(79.72, 38.66);
    private final Pose leavePose = new Pose(83.38, 58.97, Math.toRadians(90));
    public ComponentShell comps;
    public PathChain scorePreload, leave, grabPickupHp, scorePickupHp;
    public int Shots = 0;
    private TelemetryManager telemetryM;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        //scorePreload = new Path(new BezierLine(startPose, scorePosePreload));
        //scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePosePreload.getHeading());
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        grabPickupHp = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickupHpControlpoint, pickupHpPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupHpPose.getHeading())
                .build();

        scorePickupHp = follower.pathBuilder()
                .addPath(new BezierLine(pickupHpPose, scorePose))
                .setLinearHeadingInterpolation(pickupHpPose.getHeading(), scorePose.getHeading())
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, leavePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leavePose.getHeading())
                .build();
    }


    public void autonomousPathUpdate() {

        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                comps.through.InThrough();
                comps.shooter.PreTargetTo(scorePose);
                nextPathState();
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    follower.followPath(grabPickupHp);
                    nextPathState();
                }
                break;

            case 2:
                if(!follower.isBusy()){
                    follower.followPath(scorePickupHp);
                    nextPathState();
                }
                break;

            case 5:
                if(!follower.isBusy()){
                    comps.intake.StaticIntake(comps);
                    follower.followPath(leave,true);
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