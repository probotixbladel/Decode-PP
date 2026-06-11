package org.firstinspires.ftc.teamcode.drive;

import android.annotation.SuppressLint;

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
import org.firstinspires.ftc.teamcode.util.DriveByWire;

@Configurable
@TeleOp(name="PedroTeleop", group="Linear OpMode")
public class PedroTeleop extends OpMode {
    public static boolean singlePlayer = false;
    private Follower follower;
    public static Pose startingPose;
    public static ComponentShell.Alliance alliance;
    private boolean robotCentric = false;
    private TelemetryManager telemetryM;
    private DriveByWire driveByWire;
    private ComponentShell Comps;
    private volatile boolean started = false;


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        Storage.Data data = Storage.read();
        startingPose = data.storedPose;
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        alliance = data.storedAlliance;
        Comps = new ComponentShell(hardwareMap, follower, telemetryM, alliance, singlePlayer);
		driveByWire = new DriveByWire(Comps);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

	@SuppressLint("SuspiciousIndentation")
	@Override
    public void loop() {
        if (gamepad2.xWasPressed()) {
            started = true;
        }
        if (!started) {
            return;
        }


		if (singlePlayer) {
			if (gamepad1.left_stick_button) {
				driveByWire.gear = 1;
			} else {
				driveByWire.gear = 3;
			}
            } else {
			if (gamepad1.a) {
				driveByWire.gear = 3;
			} else if (gamepad1.b) {
				driveByWire.gear = 0;
			}
		}

		if (gamepad1.dpadDownWasPressed()) {
			robotCentric = !robotCentric;
		}

        double[] driveInputs = driveByWire.adjustInputs(
            -gamepad1.left_stick_x,
            -gamepad1.left_stick_y,
            -gamepad1.right_stick_x,
			gamepad1
        );



        if (robotCentric) {
            follower.setTeleOpDrive(
                -driveInputs[1],
                -driveInputs[0],
                driveInputs[2],
                true // Robot Centric
            );
        } else {
			switch (alliance) {
				case BLUE:
					follower.setTeleOpDrive(
							-driveInputs[1],
							-driveInputs[0],
							driveInputs[2],
							false // Robot Centric
					);
					break;
				case RED:
					follower.setTeleOpDrive(
							driveInputs[1],
							driveInputs[0],
							driveInputs[2],
							false // Robot Centric
					);
					break;
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


    @Override
    public void stop() {
        Storage.write(alliance, follower.getPose());
    }
}