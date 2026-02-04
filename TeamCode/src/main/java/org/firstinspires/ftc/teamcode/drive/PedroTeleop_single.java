package org.firstinspires.ftc.teamcode.drive;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.ComponentShell;
import org.firstinspires.ftc.teamcode.components.Storage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Configurable
@TeleOp(name="PedroTeleOp_single", group="Linear OpMode")
public class PedroTeleop_single extends OpMode {
    private Pose Goal;
    public static boolean SinglePlayer = true;
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    public Pose TargetPose = new Pose(70,70,Math.toRadians(270));
    public static ComponentShell.Alliance alliance;
    private boolean automatedDrive;
    private boolean robotCentric = false;
    private TelemetryManager telemetryM;
    private final PedroInputScaler scaler = new PedroInputScaler();
    private ComponentShell Comps;
    public PIDFController GoalPID;
    static public double kp = 1.35;
    static public double kd = 0.12;
    static public double kf = 0.1;
    static public int blueX = 0;
    static public int blueY = 144;
    static public int redX = 144;
    static public int redY = 144;
    static class PedroInputScaler {
        // TODO: Tune these values for your application
        // This does NOT create any mechanical advantage, it is purely for control
        private final double[] translationGears = { 0.3, 0.6, 0.8, 1.0 };
        private final double[] rotationGears = { 0.3, 0.4, 0.6, 0.75 };

        public int gear = 1; // the index of the gear in use


        // Curves exponents
        // higher: more control when slow, less control at speed
        // lower : less control when slow, more control at speed
        // n < 1 or n > 3: not recommended
        // n = 1: linear
        // 1 < n < 3: recommended
        // TODO: experiment to find your optimal value
        private static final double translationCurveExponent = 2.0;
        private static final double rotationCurveExponent = 1.2;

        // Applies a signed exponential curve to controller input.
        // This preserves the input direction while adjusting sensitivity.
        private double applyInputCurve(double input, double exponent) {
            return Math.pow(Math.abs(input), exponent) * Math.signum(input);
        }

        // Applies both gearing and input curves to translation & rotation input
        public double[] getScaledInput(double x, double y, double yaw) {
            double length = Math.sqrt((x * x + y * y));
            double scale = applyInputCurve(length, translationCurveExponent) * translationGears[gear];
            yaw = applyInputCurve(yaw, rotationCurveExponent) * rotationGears[gear];
            x *= scale;
            y *= scale;
            return new double[] {x, y, yaw};

        }
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        Storage.Data data = Storage.read();
        startingPose = data.storedPose;
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        alliance = data.storedAlliance;
        Comps = new ComponentShell(hardwareMap, follower, telemetryM, alliance, SinglePlayer);
        switch (alliance) {
            case RED:
                Goal = new Pose(redX, redY);
                break;
            case BLUE:
                Goal = new Pose(blueX, blueY);
                break;
        }

    }

    @Override
    public void start() {

        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();
        telemetryM.debug("goto: ", TargetPose);

        // switch gears
        if (SinglePlayer) {
            if (gamepad1.left_stick_button) {
                scaler.gear = 1;
            } else {
                scaler.gear = 3;
            }
        } else {
            if (gamepad1.b) {
                scaler.gear = 0;
            } else if (gamepad1.x) {
                scaler.gear = 1;
            } else if (gamepad1.y) {
                scaler.gear = 2;
            } else if (gamepad1.a) {
                scaler.gear = 3;
            }
        }
        if (gamepad1.dpadDownWasPressed()) {
            robotCentric = !robotCentric;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (!gamepad1.left_bumper || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
            Comps.shooter.Arrived();
        } else if (gamepad1.leftBumperWasPressed()) {
            double dy = Goal.getY() - follower.getPose().getY();
            double dx = Goal.getX() - follower.getPose().getX();
            double alpha = Math.atan2(dy, dx);
            double beta = alpha - Math.PI;
            /*
            follower.pathBuilder().addPath(new Path( ));
            follower.followPath(follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, follower::getPose)))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, beta, 1))
                    .build());
            */
			follower.turnTo(beta);
            automatedDrive = true;
            //Comps.shooter.PreTargetTo(TargetPose);
        }
        if (gamepad1.rightBumperWasPressed()){
            GoalPID = new PIDFController(new PIDFCoefficients(kp,0,kd,kf));
        }

        if (!automatedDrive) {
            //This is the normal version to use in the TeleOp

            double[] driveInputs = scaler.getScaledInput(
                    -gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x

            );
            if (robotCentric) {
                follower.setTeleOpDrive(
                        -driveInputs[1], //
                        -driveInputs[0], //
                        driveInputs[2],
                        true // Robot Centric
                );
            } else {
                if(gamepad1.right_bumper) {
                    double dy = Goal.getY() - follower.getPose().getY();
                    double dx = Goal.getX() - follower.getPose().getX();
                    double alpha = Math.atan2(dy, dx);
                    double beta = alpha - Math.PI;
                    GoalPID.setTargetPosition(beta);
                    GoalPID.updatePosition(follower.getHeading());

                    if(alliance == ComponentShell.Alliance.BLUE){
                        follower.setTeleOpDrive(
                                -driveInputs[1],
                                -driveInputs[0],
                                Math.min(Math.max(GoalPID.run(),-1),1),
                                false // Robot Centric
                        );
                    }
                    else if(alliance == ComponentShell.Alliance.RED){
                        follower.setTeleOpDrive(
                                driveInputs[1],
                                driveInputs[0],
                                Math.min(Math.max(GoalPID.run(),-1),1),
                                false // Robot Centric
                        );
                    }

                }
                else if(alliance == ComponentShell.Alliance.BLUE){
                    follower.setTeleOpDrive(
                            -driveInputs[1],
                            -driveInputs[0],
                            driveInputs[2],
                            false // Robot Centric
                    );
                }
                else if(alliance == ComponentShell.Alliance.RED){
                    follower.setTeleOpDrive(
                            driveInputs[1],
                            driveInputs[0],
                            driveInputs[2],
                            false // Robot Centric
                    );
                }
            }
        }
        Comps.updateTeleop(gamepad1, gamepad2);
    }

    /*public Pose NearestShot(Pose pos) {
        Pose ShootPos;
        Pose Goal = new Pose();
        switch (alliance) {
            case RED:
                Goal = new Pose(133,135);
                break;
            case BLUE:
                Goal = new Pose(11,135);
                break;
        }

        boolean InShootZone = false;
        if (pos.getY() > 72 & pos.getY() < 130) {
            double MaxDeviation = pos.getY() - 72;
            InShootZone = pos.getX() > 72 - MaxDeviation & pos.getX() < 72 + MaxDeviation;
        } else if (pos.getY() < 24 & pos.getY() > 14) {
            double MaxDeviation = 24 - pos.getY();
            InShootZone = pos.getX() > 72 - MaxDeviation & pos.getX() < 72 + MaxDeviation;
        }
        if (InShootZone) {
            if(Goal.distanceFrom(pos) < 28){
                double mult = 28/Goal.distanceFrom(pos);
                double x = Goal.getX() - ((Goal.getX() - pos.getX()) * mult);
                double y = Goal.getY() - ((Goal.getY() - pos.getY()) * mult);
                return new Pose(x, y, Math.atan2(Goal.getY() - pos.getY(), Goal.getX() - pos.getX()) + Math.PI);
            }

            return pos.withHeading(Math.atan2(Goal.getY() - pos.getY(), Goal.getX() - pos.getX()) - 0.5 * Math.PI);
        }


        if (pos.getY() >= 130 & pos.getX() < 96 & pos.getX() > 48) {
            return new Pose(pos.getX(), 130,Math.atan2(Goal.getY() - 130, Goal.getX() - pos.getX()) + Math.PI);
        }
        if (pos.getY() <= 14 & pos.getX() < 86 & pos.getX() > 58) {
            return new Pose(pos.getX(), 14,Math.atan2(Goal.getY() - 14, Goal.getX() - pos.getX()) + Math.PI);
        }

        Pose near  = ClosePoint(pos, new Pose(72,72), new Pose(29 ,113), Goal);
        Pose far   = ClosePoint(pos, new Pose(72,24), new Pose(63 ,15 ), Goal);
        Pose near1 = ClosePoint(pos, new Pose(72,72), new Pose(115,113), Goal);
        Pose far1  = ClosePoint(pos, new Pose(72,24), new Pose(81 ,15 ), Goal);

        double MinDist = near.distanceFrom(pos);
        ShootPos = near;
        if (far.distanceFrom(pos) < MinDist) {
            MinDist = far.distanceFrom(pos);
            ShootPos = far;
        }
        if (near1.distanceFrom(pos) < MinDist) {
            MinDist = near1.distanceFrom(pos);
            ShootPos = near1;
        }
        if (far1.distanceFrom(pos) < MinDist) {
            MinDist = far1.distanceFrom(pos);
            ShootPos = far1;
        }

        return ShootPos;
    }*/                                                                 // sjoerd told me too, im sowwy

    /*public Pose ClosePoint(Pose point, Pose startLine, Pose endLine, Pose goal) {
        double abX = endLine.getX() - startLine.getX();
        double abY = endLine.getY() - startLine.getY();

        double t = ((point.getX() - startLine.getX()) * abX +
                (point.getY() - startLine.getY()) * abY)
                / (abX * abX + abY * abY);

        t = Math.max(0, Math.min(1, t));

        double x = startLine.getX() + t * abX;
        double y = startLine.getY() + t * abY;

        double heading = Math.atan2(goal.getY() - y, goal.getX() - x);

        return new Pose(x, y, heading + Math.PI);
    }*/
}