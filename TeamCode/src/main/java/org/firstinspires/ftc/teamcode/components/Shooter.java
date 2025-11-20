package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Shooter {
    private HardwareMap hardwareMap;
    public DcMotorEx ShooterLeft;
    public DcMotorEx ShooterRight;
    public ShooterState state = ShooterState.LOW;
    public static double TargetVel = 1800;
    public static double MaxDeviation = 150;
    public double CurrentVel = 0;
    public enum ShooterState {
        READY,
        HIGH,
        LOW,
    }
    public Shooter(HardwareMap hwm) {
        this.hardwareMap = hwm;
        ShooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        ShooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        //ShooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //ShooterRight.setDirection(DcMotorSimple.Direction.FORWARD);
        ShooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterLeft.setVelocityPIDFCoefficients( 0.1, 0, 0, 0.4);
        ShooterRight.setVelocityPIDFCoefficients(0.1, 0, 0, 0.4);
        ShooterLeft.setVelocity(TargetVel);
        ShooterRight.setVelocity(TargetVel);
    }

    public void update(){
        CurrentVel = ShooterLeft.getVelocity();
        if (CurrentVel < TargetVel - MaxDeviation) {
            state = ShooterState.LOW;
        } else if (CurrentVel > TargetVel - MaxDeviation) {
            state = ShooterState.HIGH;
        } else {
            state = ShooterState.READY;
        }
    }

}