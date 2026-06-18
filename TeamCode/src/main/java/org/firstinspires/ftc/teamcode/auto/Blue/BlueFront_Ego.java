package org.firstinspires.ftc.teamcode.auto.Blue;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import java.util.List;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.SOTMInterpolator;
import org.firstinspires.ftc.teamcode.components.ComponentShell;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Pusher;
import org.firstinspires.ftc.teamcode.components.Storage;
import org.firstinspires.ftc.teamcode.components.Through;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Configurable
@Autonomous(name = "BlueFront_Ego")
public class BlueFront_Ego extends OpMode {
    private Follower follower;
    List<LynxModule> allHubs;
    public ElapsedTime Timer = new ElapsedTime();
    private Timer pathTimer, actionTimer, opmodeTimer;
    public static double gateTime = 7;
    public static double shootigDrivePower = 0.4;
    private int pathState;
    private final Pose startPose = new Pose(17.43, 120.37, Math.toRadians(-36));
    private final Pose scorePoseStart = new Pose(37.13, 107.03, Math.toRadians(-44));
    private final Pose scorePoseFinal = new Pose(51.32, 93.54, Math.toRadians(-46));
    private final Pose pickup1Pose = new Pose(17.44, 83.92, Math.toRadians(180));
    private final Pose pickup1Controlpoint = new Pose(53.92, 83.07);
    private final Pose pickup1PoseHalfway = new Pose(45, 83.92, Math.toRadians(180));
    private final Pose scorePose = new Pose(51.60, 93.92, Math.toRadians(-46));
    private final Pose pickup2Pose = new Pose(16.72, 59.93, Math.toRadians(180));
    private final Pose pickup2Controlpoint = new Pose(65.56, 61.39);
    private final Pose pickup2PoseHalfway = new Pose(45, 59.93, Math.toRadians(180));
    private final Pose pickupGatePose = new Pose(11.21, 60.61, Math.toRadians(143));
    private final Pose pickupGateControlpoint = new Pose(53.96, 70.92);
    private final Pose pickup3Pose = new Pose(23.58, 35.08, Math.toRadians(180));
    private final Pose pickup3Controlpoint = new Pose(61.04, 32.74);
    private final Pose pickup3PoseHalfway = new Pose(45, 35.08, Math.toRadians(180));
    private final Pose leavePose = new Pose(62.25, 64.65, Math.toRadians(-90));
    private final Pose leaveControlpose = new Pose(73.96, 77.51);
    public ComponentShell comps;
    public PathChain scorePreloadStart, scorePreloadFinal, grabGate, scoreGate, grabPickup1Part1, grabPickup1Part2, grabPickup2Part1, grabPickup2Part2, scorePickup1, scorePickup2, leave, grabPickup3Part1, grabPickup3Part2, scorePickup3;
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

        grabPickup1Part1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePoseFinal, pickup1Controlpoint, pickup1PoseHalfway))
                .setLinearHeadingInterpolation(scorePoseFinal.getHeading(), pickup1PoseHalfway.getHeading())
                .build();

        grabPickup1Part2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1PoseHalfway, pickup1Pose))
                .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2Part1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup2Controlpoint, pickup2PoseHalfway))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2PoseHalfway.getHeading())
                .build();

        grabPickup2Part2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PoseHalfway, pickup2Pose))
                .setConstantHeadingInterpolation(pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabGate = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickupGateControlpoint, pickupGatePose))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, 0.5, HeadingInterpolator.linear(scorePose.getHeading(), pickupGatePose.getHeading())),
                        new HeadingInterpolator.PiecewiseNode(0.5, 1, HeadingInterpolator.constant(pickupGatePose.getHeading()))))
                .build();

        scoreGate = follower.pathBuilder()
                .addPath(new BezierLine(pickupGatePose, scorePose))
                .setLinearHeadingInterpolation(pickupGatePose.getHeading(), pickupGatePose.getHeading())
                .build();

        grabPickup3Part1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup3Controlpoint, pickup3PoseHalfway))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3PoseHalfway.getHeading())
                .build();

        grabPickup3Part2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3PoseHalfway, pickup3Pose))
                .setConstantHeadingInterpolation(pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, leaveControlpose, leavePose))
                //.setLinearHeadingInterpolation(scorePose.getHeading(), leavePose.getHeading())
                .setHeadingInterpolation(new SOTMInterpolator().giveInfo(follower, comps))
                .build();
    }


    public void autonomousPathUpdate() {

        switch (pathState) {
            case 0:
                follower.followPath(scorePreloadStart);
                comps.through.state = Through.ThroughState.OFF;
                comps.intake.state = Intake.IntakeState.INTAKE;
                comps.shooter.PreTargetTo(scorePoseStart);
                nextPathState();
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    follower.followPath(scorePreloadFinal, shootigDrivePower, false);
                    nextPathState();
                }
                break;

            case 2:
                comps.AutoShooterStart();
                if(!follower.isBusy() && comps.FinishedShooting(3)){
                    follower.followPath(grabPickup1Part1);
                    nextPathState();
                }
                break;

            case 3:
                if(!follower.isBusy()){
                    follower.followPath(grabPickup1Part2, 0.6, false);
                    nextPathState();
                }
                break;

            case 4:
                if(!follower.isBusy()){
                    comps.ResetShootNum();
                    follower.followPath(scorePickup1,true);
                    nextPathState();
                }
                break;

            case 5:
                if(!follower.isBusy()){
                    comps.AutoShooterStart();
                    follower.holdPoint(scorePose.withHeading(comps.shooter.shootInDirection(comps)));
                    if(comps.FinishedShooting(3) && comps.pusher.state == Pusher.PushState.RETURNING){
                        follower.followPath(grabPickup2Part1, 1, false);
                        nextPathState();
                    }
                }
                break;

            case 6:
                if(!follower.isBusy()){
                    follower.followPath(grabPickup2Part2,0.6, false);
                    nextPathState();
                }
                break;

            case 7:
                if(!follower.isBusy()){
                    follower.followPath(scorePickup2, true);
                    comps.ResetShootNum();
                    nextPathState();
                }
                break;

            case 8:
                if(!follower.isBusy()) {
                    comps.AutoShooterStart();
                    follower.holdPoint(scorePose.withHeading(comps.shooter.shootInDirection(comps)));
                    if(comps.FinishedShooting(3) && comps.pusher.state == Pusher.PushState.RETURNING){
                        actionTimer.resetTimer();
                        follower.followPath(grabGate, true);
                        nextPathState();
                    }
                }
                break;

            case 9:
                if(comps.detector.thirdDetecting ||actionTimer.getElapsedTimeSeconds() > gateTime){
                    follower.followPath(scoreGate);
                    comps.ResetShootNum();
                    nextPathState();
                }
                break;

            case 10:
                if(!follower.isBusy()){
                    comps.AutoShooterStart();
                    follower.holdPoint(scorePose.withHeading(comps.shooter.shootInDirection(comps)));
                    if(comps.FinishedShooting(3) && comps.pusher.state == Pusher.PushState.RETURNING){
                        follower.followPath(grabPickup3Part1, false);
                        nextPathState();
                    }
                }
                break;

            case 11:
                if(!follower.isBusy()){
                    follower.followPath(grabPickup3Part2, 0.6, true);
                    nextPathState();
                }
                break;

            case 12:
                if(!follower.isBusy()){
                    follower.followPath(scorePickup3);
                    comps.ResetShootNum();
                    nextPathState();
                }
                break;

            case 13:
                if(!follower.isBusy()){
                    comps.AutoShooterStart();
                    follower.followPath(leave);
                    setPathState(-1);
                }

        }
    }

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
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
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
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