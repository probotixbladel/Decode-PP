package org.firstinspires.ftc.teamcode.drive;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.ComponentShell;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp(name="TeleOp Red Back", group="Linear OpMode")
public class PedroTeleop_Red_Back extends OpMode {
    private Follower follower;
    public static boolean SinglePlayer = true;
    public static Pose startingPose = new Pose(87.0, 9, Math.toRadians(270)); //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private boolean robotcentric = true;
    private Supplier<PathChain> pathChainClose;
    private Supplier<PathChain> pathChainFar;
    private TelemetryManager telemetryM;
    private final PedroInputScaler scaler = new PedroInputScaler();
    private ComponentShell Comps;

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
        private static final double rotationCurveExponent = 2.0;

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
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Comps = new ComponentShell(hardwareMap, follower, telemetryM, ComponentShell.Alliance.BLUE, SinglePlayer);
        pathChainClose = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(104, 110))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(225), 0.8))
                .build();

        pathChainFar = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(86, 95))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(225), 0.8))
                .build();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

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
            robotcentric = !robotcentric;
        }



        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.leftBumperWasPressed() || gamepad1.rightBumperWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        } else if (gamepad1.rightBumperWasPressed()) {
            follower.followPath(pathChainFar.get());
            automatedDrive = true;
        } else if (gamepad1.leftBumperWasPressed()) {
            follower.followPath(pathChainClose.get());
            automatedDrive = true;
        }

        if (!automatedDrive) {
            //This is the normal version to use in the TeleOp
            double[] driveInputs = scaler.getScaledInput(
                    -gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x
            );
            if (robotcentric) {
                follower.setTeleOpDrive(
                        driveInputs[1],
                        driveInputs[0],
                        driveInputs[2],
                        true // Robot Centric
                );
            } else {
                follower.setTeleOpDrive(
                        -driveInputs[1],
                        -driveInputs[0],
                        driveInputs[2],
                        false // Robot Centric
                );
            }
        }

        //telemetryM.debug("position", follower.getPose());
        //telemetryM.debug("velocity", follower.getVelocity());
        //telemetryM.debug("automatedDrive", automatedDrive);

        Comps.updateTeleop(gamepad1, gamepad2);
    }
}