package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.ComponentShell;
import org.firstinspires.ftc.teamcode.components.Pusher;
import org.firstinspires.ftc.teamcode.components.Storage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Configurable
@Autonomous(name = "BlueBack_ph3", group = "Examples")
public class BlueBack_ph3 extends OpMode {
    private Follower follower;
    public ElapsedTime Timer = new ElapsedTime();
    private Timer pathTimer, actionTimer, opmodeTimer;
    public static double scoopTime = 2;
    private int pathState;
    private final Pose startPose = new Pose(56, 7, Math.toRadians(-90)); // Starting pose for our robot
    private final Pose scorePosePreload = new Pose(59, 10, Math.toRadians(290)); // Scoring Pose of our robot for the preload. It is facing the goal at a 290 degree angle.
    private final Pose pickup1Setup = new Pose(11, 21, Math.toRadians(200)); // Setup to pickup balls in the hp zone
    private final Pose pickup1Pose = new Pose(11, 12, Math.toRadians(200));// balls in the hp zone
    private final Pose scorePose1 = new Pose(59, 10, Math.toRadians(290)); // Scoring Pose of our robot for the first pickup. It is facing the goal at a 290 degree angle.
    private final Pose pickup2Setup = new Pose(42, 36, Math.toRadians(180)); // Setup to pickup the lowest set of balls
    private final Pose pickup2Pose = new Pose(15, 36, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose scorePose2 = new Pose(59, 10, Math.toRadians(290)); // Scoring Pose of our robot for the second pickup. It is facing the goal at a 290 degree angle.
    private final Pose scoopSetup = new Pose(12, 50, Math.toRadians(240)); // Setup to scoop remaining balls in the hp zone
    private final Pose scoopPose = new Pose(11, 14, Math.toRadians(240)); // Pose to scoop remaining balls in the hp zone
    private final Pose scorePose3 = new Pose(59, 10, Math.toRadians(290)); // Scoring Pose of our robot for the first pickup. It is facing the goal at a 290 degree angle.
    private final Pose leaveTriangle = new Pose(60, 35, Math.toRadians(0)); // Leave small triangle
    public Path scorePreload;
    public ComponentShell comps;
    public PathChain grabPickup1, grabPickup2, scorePickup1, scorePickup2, scorePickup3, grabPickupSetup1, grabPickupSetup2, leave, grabScoopSetup, grabScoop;
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

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose1))
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

        grabScoopSetup = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, scoopSetup))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), scoopSetup.getHeading())
                .build();

        grabScoop = follower.pathBuilder()
                .addPath(new BezierLine(scoopSetup, scoopPose))
                .setLinearHeadingInterpolation(scoopSetup.getHeading(), scoopPose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scoopPose, scorePose3))
                .setLinearHeadingInterpolation(scoopPose.getHeading(), scorePose3.getHeading())
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, leaveTriangle))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), leaveTriangle.getHeading())
                .build();
    }


    public void autonomousPathUpdate() {

        switch (pathState) {
            case 0:
                comps.through.InThrough(comps);
                comps.shooter.PreTargetTo(scorePosePreload);
                follower.followPath(scorePreload);
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
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    setPathState(6);
                }
                break;

            case 6:
                if(!follower.isBusy()){
                    comps.ResetShootNum();
                    comps.shooter.Arived();
                    setPathState(7);
                }
                break;

            case 7:
                comps.AutoShooterStart();
                if(comps.FinishedShooting(3) && (comps.pusher.state == Pusher.PushState.WAITING || comps.pusher.state == Pusher.PushState.RELOADING)){
                    comps.shooter.PreTargetTo(scorePose2);
                    setPathState(8);
                }
                break;

            case 8:
                if(!follower.isBusy()){
                    follower.followPath(grabPickupSetup2,true);
                    setPathState(9);
                }
                break;

            case 9:
                if(!follower.isBusy()){
                    comps.intake.TakeIn(comps);
                    follower.followPath(grabPickup2, 1, true);
                    setPathState(10);
                }
                break;

            case 10:
                if(!follower.isBusy()) {
                    comps.intake.StaticIntake();
                    follower.followPath(scorePickup2, true);
                    setPathState(11);
                }
                break;

            case 11:
                if(!follower.isBusy()){
                    comps.ResetShootNum();
                    comps.shooter.Arived();
                    setPathState(12);
                }
                break;

            case 12:
                comps.AutoShooterStart();
                if(comps.FinishedShooting(3) && (comps.pusher.state == Pusher.PushState.WAITING || comps.pusher.state == Pusher.PushState.RELOADING)){
                    setPathState(13);
                }
                break;

            case 13:
                comps.intake.StaticIntake();
                comps.through.StaticThrough(comps);
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
