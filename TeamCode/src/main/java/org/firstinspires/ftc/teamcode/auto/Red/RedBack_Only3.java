package org.firstinspires.ftc.teamcode.auto.Red;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.ComponentShell;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Pusher;
import org.firstinspires.ftc.teamcode.components.Storage;
import org.firstinspires.ftc.teamcode.components.Through;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Configurable
@Disabled
@Autonomous(name = "RedBack_Only3")
public class RedBack_Only3 extends OpMode {
    private Follower follower;
    List<LynxModule> allHubs;
    public ElapsedTime Timer = new ElapsedTime();
    private Timer pathTimer, actionTimer, opmodeTimer;
    public static double HpTime = 6;
    private int pathState;
    public static double fieldLength = 141.5;
    private final Pose startPose = new Pose(55.45, 8.28, Math.toRadians(-90)).mirror(fieldLength);
    private final Pose scorePose = new Pose(59.65, 16.65, Math.toRadians(-66)).mirror(fieldLength);
    private final Pose pickup3Pose = new Pose(20.73, 38.36, Math.toRadians(-180)).mirror(fieldLength);
    private final Pose pickup3Controlpoint = new Pose(5, 56).mirror(fieldLength);
    private final Pose pickup3PoseHalfway = new Pose(45,38.36, Math.toRadians(-180)).mirror(fieldLength);
    private final Pose leavePose = new Pose(60.62, 58.97, Math.toRadians(-90)).mirror(fieldLength);
    public ComponentShell comps;
    public PathChain scorePreload, leave, grabPickup3Part1, grabPickup3Part2, scorePickup3;
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
                .addPath(new BezierLine(scorePose, leavePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leavePose.getHeading())
                .build();
    }


    public void autonomousPathUpdate() {

        switch (pathState) {
            case 0:
                comps.intake.state = Intake.IntakeState.INTAKE;
                follower.followPath(scorePreload,1, false);
                comps.through.state = Through.ThroughState.OFF;
                comps.ResetShootNum();
                nextPathState();
                break;

            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()){
                    comps.AutoShooterStart();
                    follower.holdPoint(scorePose.withHeading(comps.shooter.shootInDirection(comps)));
                    if(comps.FinishedShooting(3) && comps.pusher.state == Pusher.PushState.RETURNING) {
                        follower.followPath(grabPickup3Part1, 0.6, false);
                        nextPathState();
                    }
                }
                break;

            case 2:
                if(!follower.isBusy()){
                    follower.followPath(grabPickup3Part2, 1, false);
                    nextPathState();
                }
                break;

            case 3:
                if(!follower.isBusy()){
                    follower.followPath(scorePickup3,true);
                    comps.ResetShootNum();
                    nextPathState();
                }
                break;

            case 6:
                if(!follower.isBusy()){
                    comps.AutoShooterStart();
                    follower.holdPoint(scorePose.withHeading(comps.shooter.shootInDirection(comps)));
                    if(comps.FinishedShooting(3) && comps.pusher.state == Pusher.PushState.RETURNING){
                        follower.followPath(leave);
                        setPathState(-1);
                    }
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
        comps = new ComponentShell(hardwareMap, follower, telemetryM, ComponentShell.Alliance.RED, true);
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
        Storage.write(ComponentShell.Alliance.RED, follower.getPose());
    }
}