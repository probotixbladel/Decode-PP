package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Shooter {
    private HardwareMap hardwareMap;
    public DcMotorEx ShooterLeft;
    //public DcMotorEx ShooterRight;
    public ShooterState state = ShooterState.LOW;
    public static double TargetVel = 1250;
    public static double MaxDeviation = 75;
    public static double MinDeviation = 75;
    public double CurrentVel = 0;

    public static double P = 12.5;
    public static double D = 0.0;
    public static double F = 20.0;
    private double lP = P;
    private double lD = D;
    private double lF = F;

    public enum ShooterState {
        READY,
        HIGH,
        LOW,
    }

    public Shooter(HardwareMap hwm) {
        this.hardwareMap = hwm;
        ShooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        ShooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        ShooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterLeft.setVelocityPIDFCoefficients( P, 0, D, F);
    }

    public void update(){
        if (P != lP | D != lD | F != lF){
            ShooterLeft.setVelocityPIDFCoefficients( P, 0, D, F);
            lP = P;
            lD = D;
            lF = F;
        }


        ShooterLeft.setVelocity(TargetVel);
        //ShooterRight.setVelocity(TargetVel);
        CurrentVel = ShooterLeft.getVelocity();
        if (CurrentVel < TargetVel - MinDeviation) {
            state = ShooterState.LOW;
        } else if (CurrentVel > TargetVel + MaxDeviation) {
            state = ShooterState.HIGH;
        } else {
            state = ShooterState.READY;
        }
    }

}
