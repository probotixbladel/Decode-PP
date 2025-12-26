package org.firstinspires.ftc.teamcode.auto;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.components.ComponentShell;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Pusher;
import org.firstinspires.ftc.teamcode.components.Shooter;
import org.firstinspires.ftc.teamcode.components.Through;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Goal Auto", group = "Examples")
public class BlueGoalAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    //private final Pose startPose = new Pose(28.5, 128, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose startPose = new Pose(25, 131, Math.toRadians(-36)); //See ExampleAuto to understand how to use this
    private final Pose scorePose = new Pose(59, 85, Math.toRadians(-48)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Setup = new Pose(50, 85, Math.toRadians(180)); // Setup to pickup the highest set of balls
    private final Pose pickup1Pose = new Pose(19, 85, Math.toRadians(180));// Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Setup = new Pose(50, 60, Math.toRadians(180)); // Setup to pickup the middle set of balls
    private final Pose pickup2Pose = new Pose(19, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Setup = new Pose(50, 36, Math.toRadians(180)); // Setup to pickup the lowest set of balls
    private final Pose pickup3Pose = new Pose(19, 36, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    public PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;
    public Path scorePreload;
    public Shooter shooter;
    public Pusher pusher;
    public Intake intake;
    public Through through;
    public ComponentShell comps;
    // for shoot pdf
    private double lP = shooter.P;
    private double lD = shooter.D;
    private double lF = shooter.F;



    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Setup))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addPath(new BezierLine(pickup1Setup, pickup1Pose))
                .setLinearHeadingInterpolation(pickup1Setup.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Setup))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .addPath(new BezierLine(pickup2Setup, pickup2Pose))
                .setLinearHeadingInterpolation(pickup2Setup.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Setup))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .addPath(new BezierLine(pickup3Setup, pickup3Pose))
                .setLinearHeadingInterpolation(pickup3Setup.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }
    private void Shoot(int numShots){
        for (int i = 0; i < numShots; i++) {
            if(comps.shooter.state != Shooter.ShooterState.READY){
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    return; // Exit if interrupted
                }
            }
            pusher.Pusher.setPosition(pusher.Push);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                return; // Exit if interrupted
            }
            pusher.Pusher.setPosition(pusher.Wait);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                return; // Exit if interrupted
            }
            through.Through.setPower(through.in_power);
            intake.intake.setPower(intake.intake_power);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                return; // Exit if interrupted
            }
            through.Through.setPower(through.static_power);
            intake.intake.setPower(intake.static_power);
        }
    }
    public void autonomousPathUpdate() {

        /* SHOOTER TO SHOOT BALLS */

        if (shooter.P != lP | shooter.D != lD | shooter.F != lF){
            shooter.ShooterLeft.setVelocityPIDFCoefficients( shooter.P, 0, shooter.D, shooter.F);
           // shooter.ShooterRight.setVelocityPIDFCoefficients(shooter.P, 0, shooter.D, shooter.F);
            lP = shooter.P;
            lD = shooter.D;
            lF = shooter.F;
            /*
            shooter.ShooterLeft.setVelocity(shooter.TargetVel);
           // shooter.ShooterRight.setVelocity(shooter.TargetVel);
            shooter.CurrentVel = shooter.ShooterLeft.getVelocity();
            if (shooter.CurrentVel < shooter.TargetVel - shooter.MinDeviation) {
                shooter.state = Shooter.ShooterState.LOW;
            } else if (shooter.CurrentVel > shooter.TargetVel + shooter.MaxDeviation) {
                shooter.state = Shooter.ShooterState.HIGH;
            } else {
                shooter.state = Shooter.ShooterState.READY;
            }
            */
        }

        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);

                /* SHOOT PRELOAD */
                Shoot(3);

                setPathState(1);
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy()) {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */
                // This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position
                if (!follower.isBusy()) {

                    /* GRAB PICKUP1 */
                    intake.intake.setPower(intake.intake_power);
                    through.Through.setPower(through.in_power);

                    // Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample
                    follower.followPath(grabPickup1, true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {

                    /* STOP GRABBING BALLS */
                    intake.intake.setPower(intake.static_power);
                    through.Through.setPower(through.static_power);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1, true);

                    /* SHOOT 1 */
                    Shoot(3);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {

                    /* GRAB PICKUP2 */
                    intake.intake.setPower(intake.intake_power);
                    through.Through.setPower(through.in_power);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2, true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* STOP GRABBING BALLS */
                    intake.intake.setPower(intake.static_power);
                    through.Through.setPower(through.static_power);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2, true);

                    /* SHOOT 2 */
                    Shoot(3);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* GRAB PICKUP3 */
                    intake.intake.setPower(intake.intake_power);
                    through.Through.setPower(through.in_power);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3, true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* STOP GRABBING BALLS */
                    intake.intake.setPower(intake.static_power);
                    through.Through.setPower(through.static_power);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);

                    /* SHOOT 2 */
                    Shoot(3);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        //createPoses();
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}