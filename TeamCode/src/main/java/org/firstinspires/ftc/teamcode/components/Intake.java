package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Intake {
	public DcMotorEx intake;
	public static double intakePower = 0.8;
	public static double intakePowerOverCurrent = 0.6;
    public static double outtakePower = -0.4;
    public static double staticPower = 0.3;
    public Intake.IntakeState state = IntakeState.INTAKE;
    public enum IntakeState {
        INTAKE,
        OUTTAKE
    }

    public Intake(HardwareMap hwm) {
		intake = hwm.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update(ComponentShell Comps){
        switch(state){
            case INTAKE:
                if (Comps.detector.fourthDetecting) {
                    if (intake.getPower() != outtakePower) {
                        intake.setPower(outtakePower);
                    }
                    return;
                }


                if (Comps.pusher.state == Pusher.PushState.WAITING || Comps.pusher.state == Pusher.PushState.RELOADING) {
                    if (Comps.floodgate.floodgateCurrent > 18) {
                        if (intake.getPower() != intakePowerOverCurrent) {
                            intake.setPower(intakePowerOverCurrent);
                        }
                    } else {
                        if (intake.getPower() != intakePower) {
                            intake.setPower(intakePower);
                        }
                    }
                } else {
                    intake.setPower(0);
                }
                break;

            case OUTTAKE:
                if (intake.getPower() != outtakePower) {
                    intake.setPower(outtakePower);
                }
                break;
        }
    }

    public void forceIntake() {
        intake.setPower(intakePower);
        state = IntakeState.INTAKE;
    }

    /*public void TakeIn(ComponentShell Comps) {
        if (Comps.detector.fourthDetecting) {
            if (intake.getPower() != outtakePower) {
                intake.setPower(outtakePower);
            }
            state = Intake.IntakeState.OUTTAKE;
            return;
        }

		if (Comps.pusher.state == Pusher.PushState.WAITING || Comps.pusher.state == Pusher.PushState.RELOADING) {
			state = IntakeState.INTAKE;
			if (Comps.floodgate.floodgateCurrent > 18) {
                if (intake.getPower() != intakePowerOverCurrent) {
                    intake.setPower(intakePowerOverCurrent);
                }
			} else {
                if (intake.getPower() != intakePower) {
                    intake.setPower(intakePower);
                }
			}
        }
        else {
            intake.setPower(0);
        }
    }

    public void TakeOut(ComponentShell Comps) {
        state = Intake.IntakeState.OUTTAKE;
        if (intake.getPower() != outtakePower) {
            intake.setPower(outtakePower);
        }
    }
    public void StaticIntake(ComponentShell Comps) {
        if (Comps.detector.fourthDetecting) {
            if (intake.getPower() != outtakePower) {
                intake.setPower(outtakePower);
            }
            state = Intake.IntakeState.OUTTAKE;
            return;
        }
    }*/

}