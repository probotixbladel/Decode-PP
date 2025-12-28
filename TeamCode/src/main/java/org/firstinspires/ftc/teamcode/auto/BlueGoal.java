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
    public Path scorePreload;
    public ComponentShell comps;
    public double Shots = 0;
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
                setPathState(1);
                comps.shooter.PreTargetTo(scorePose);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                Shots = 3;
                comps.intake.TakeIn();
                comps.shooter.Arived();
                break;
            case 2:
                if (comps.pusher.AttemptPush(comps)) {
                    Shots -= 1;
                }
                comps.through.InThrough(comps);
                if (Shots <= 0) {
                    setPathState(-1);
                }
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
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
        telemetryM.update();
        follower.update();
        autonomousPathUpdate();
        comps.update();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
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
