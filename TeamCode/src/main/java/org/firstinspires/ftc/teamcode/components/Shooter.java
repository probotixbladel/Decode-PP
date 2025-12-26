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
    public static double CloseVel = 1100;
    public static double FarVel = 1250;
    public static double MaxDeviation = 75;
    public static double MinDeviation = 75;
    public double CurrentVel = 0;

    public static double P = 100;
    public static double D = 0.0;
    public static double F = 14.0;
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
        // ShooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        //ShooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        ShooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        ShooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //ShooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //ShooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterLeft.setVelocityPIDFCoefficients( P, 0, D, F);
        //hooterRight.setVelocityPIDFCoefficients(P, 0, D, F);


    }

    public void ChangeShooterSpeed() {
        if (TargetVel == CloseVel) {
            TargetVel = FarVel;
        } else {
            TargetVel = CloseVel;
        }
    }

    public void update(){
        if (P != lP | D != lD | F != lF){
            ShooterLeft.setVelocityPIDFCoefficients( P, 0, D, F);
            //ShooterRight.setVelocityPIDFCoefficients(P, 0, D, F);
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
