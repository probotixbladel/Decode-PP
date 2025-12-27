package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.geometry.Pose;


@Configurable
public class Shooter {
    private HardwareMap hardwareMap;
    public DcMotorEx ShooterLeft;
    //public DcMotorEx ShooterRight;
    public ShooterState state = ShooterState.LOW;
    public static double TargetVel = 1250;
    public static double MaxSpeed = 75;
    public static double MinSpeed = 75;
    public double CurrentVel = 0;
    public static double[][] MinPoints = {
            {50,  1020},
            {60,  980},
            {70,  950},
            {80,  930},
            {90,  940},
            {100, 950},
            {110, 960},
            {120, 990},
            {130, 1030},
            {140, 1040},
            {150, 1050},
            {160, 1100},
            {170, 1110},
            {180, 1130},
            {190, 1130},
            {200, 1171}

    };
    public static double[][] MaxPoints = {
            {50,  1050},
            {60,  1050},
            {70,  1060},
            {80,  1060},
            {90,  1090},
            {100, 1100},
            {110, 1110},
            {120, 1120},
            {130, 1160},
            {140, 1180},
            {150, 1180},
            {160, 1210},
            {170, 1240},
            {180, 1270},
            {190, 1270},
            {200, 1300}
    };
    public static double P = 100.0;
    public static double D = 0.0;
    public static double F = 14.0;
    private double lP = P;
    private double lD = D;
    private double lF = F;
    public static double MinToMax = 0.7;
    private Pose ShootTo;

    public enum ShooterState {
        READY,
        HIGH,
        LOW,
    }


    public Shooter(HardwareMap hwm, ComponentShell.Alliance al) {
        this.hardwareMap = hwm;
        ShooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        ShooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        ShooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterLeft.setVelocityPIDFCoefficients(P, 0, D, F);
        switch (al) {
            case RED:
                ShootTo = new Pose(133,135);
            case BLUE:
                ShootTo = new Pose(11,135);
        }

    }

    public static double interpolate(double[][] p, double x) {
        int i = 0;

        // Find the segment containing x
        while (i < p.length - 1 && x > p[i + 1][0]) i++;

        // Clamp to bounds
        if (i >= p.length - 1) return p[p.length - 1][1];
        if (x <= p[0][0]) return p[0][1];

        double x0 = p[i][0];
        double y0 = p[i][1];
        double x1 = p[i + 1][0];
        double y1 = p[i + 1][1];

        // Linear interpolation
        return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
    }

    public void setSpeeds(Pose RobotPos) {
        double distance = (Math.sqrt(Math.pow(RobotPos.getY() - ShootTo.getY(), 2) + Math.pow(RobotPos.getX() - ShootTo.getX(), 2)) - 8) * 2.54;
        MaxSpeed = interpolate(MaxPoints, distance)-5;
        MinSpeed = interpolate(MinPoints, distance)+5;
        TargetVel = MinSpeed + (MaxSpeed - MinSpeed) * MinToMax;
    }

    public void update(){
        if (P != lP | D != lD | F != lF){
            ShooterLeft.setVelocityPIDFCoefficients( P, 0, D, F);
            lP = P;
            lD = D;
            lF = F;
        }


        ShooterLeft.setVelocity(TargetVel);
        CurrentVel = ShooterLeft.getVelocity();
        if (CurrentVel < MinSpeed) {
            state = ShooterState.LOW;
        } else if (CurrentVel > MaxSpeed) {
            state = ShooterState.HIGH;
        } else {
            state = ShooterState.READY;
        }
    }

}
