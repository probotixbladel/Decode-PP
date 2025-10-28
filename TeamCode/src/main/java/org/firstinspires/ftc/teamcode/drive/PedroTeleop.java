package org.firstinspires.ftc.teamcode.drive;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class PedroTeleop extends OpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(85.0, 8, Math.toRadians(90)); //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    public DcMotorEx intake;
    private boolean robotcentric = true;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private final PedroInputScaler scaler = new PedroInputScaler();

    static class PedroInputScaler {
        // TODO: Tune these values for your application
        // This does NOT create any mechanical advantage, it is purely for control
        private final double[] translationGears = { 0.3, 0.6, 0.8, 1.0 };
        private final double[] rotationGears = { 0.3, 0.6, 0.8, 1.0 };

        public int gear = 1; // the index of the gear in use


        // Curves exponents
        // higher: more control at slow, less control at speed
        // lower : less control at slow, more control at speed
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
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(60, 95))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(135), 0.8))
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
        if (gamepad1.a) {
            scaler.gear = 0;
        } else if (gamepad1.b) {
            scaler.gear = 1;
        } else if (gamepad1.x) {
            scaler.gear = 2;
        } else if (gamepad1.y) {
            scaler.gear = 3;
        }
        if (gamepad1.dpadDownWasPressed()) {
            robotcentric = !robotcentric;
        }

        //Automated PathFollowing
        if (gamepad1.rightBumperWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.leftBumperWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
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
        intake.setPower(gamepad1.right_trigger);

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}