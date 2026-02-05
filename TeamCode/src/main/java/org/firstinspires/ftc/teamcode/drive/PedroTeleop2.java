package org.firstinspires.ftc.teamcode.drive;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.ComponentShell;
import org.firstinspires.ftc.teamcode.components.Storage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name="PedroTeleOp2", group="Linear OpMode")
public class PedroTeleop2 extends OpMode {
    private Pose Goal;
    public static boolean SinglePlayer = false;
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

        public int gear = 3; // the index of the gear in use


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
        telemetryM.debug("goto: ", TargetPose);

        // switch gears
        if (SinglePlayer) {
            if (gamepad1.left_stick_button) {
                scaler.gear = 1;
            } else {
                scaler.gear = 3;
            }
        } else {
            if (gamepad1.a) {
                scaler.gear = 3;
            } else if (gamepad1.b) {
                scaler.gear = 0;
            }
            /*
            if (gamepad1.b) {
                scaler.gear = 0;
            } else if (gamepad1.x) {
                scaler.gear = 1;
            } else if (gamepad1.y) {
                scaler.gear = 2;
            } else if (gamepad1.a) {
                scaler.gear = 3;
            }
            */
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
            follower.turnTo(beta);
            automatedDrive = true;
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
                    driveInputs[2] = Math.min(Math.max(GoalPID.run(),-1),1);
                }

                if(alliance == ComponentShell.Alliance.BLUE){
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

		if (Comps.floodgate.floodgateCurrent > 17) {
			follower.setMaxPower(0.6);
			if (Comps.floodgate.floodgateCurrent > 20) {
				follower.setMaxPower(0.2);
			}
		} else {
			follower.setMaxPower(1);
		}

		follower.update();
		telemetryM.update();
    }
}