package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.components.ComponentShell;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "BlueGoal", group = "Examples")
public class BlueGoal extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(25, 131, Math.toRadians(-36)); //See ExampleAuto to understand how to use this
    private final Pose scorePose = new Pose(59, 85, Math.toRadians(-48)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Setup = new Pose(50, 85, Math.toRadians(180)); // Setup to pickup the highest set of balls
    private final Pose pickup1Pose = new Pose(19, 85, Math.toRadians(180));// Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Setup = new Pose(50, 60, Math.toRadians(180)); // Setup to pickup the middle set of balls
    private final Pose pickup2Pose = new Pose(19, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Setup = new Pose(50, 36, Math.toRadians(180)); // Setup to pickup the lowest set of balls
    private final Pose pickup3Pose = new Pose(19, 36, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    public Path scorePreload;
    public ComponentShell comps;
    public int Shots = 0;
    private TelemetryManager telemetryM;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
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
                if(comps.FinishedShooting(3))
                {
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
        Carry.pos = follower.getPose();
        Carry.alliance = ComponentShell.Alliance.BLUE;
    }
}
