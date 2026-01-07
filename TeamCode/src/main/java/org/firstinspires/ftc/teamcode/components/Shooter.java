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
    public static double MaxSpeed = 1300;
    public static double MinSpeed = 1200;
    public double CurrentVel = 0;
    public static boolean PreTargeting = false;
    public static double[][] MinPoints = {  // data min snelheid
            {60,1000},
            {70,1000},
            {80,960},
            {90,950},
            {100,960},
            {110,960},
            {120,980},
            {130,1010},
            {140,1020},
            {150,1040},
            {160,1060},
            {170,1080},
            {180,1120},
            {190,1140},
            {200,1160},
            {210,1190},
            {220,1210},
            {230,1230},
            {240,1270},
            {250,1280},
            {260,1320},
            {270,1360},
            {280,1370},
            {290,1390},
            {300,1410},
            {310,1430}
            //{320,1440},
            //{330,1480},
            //{340,1510},
            //{350,1520},
            //{360,1520},
            //{370,1525},
            //{380,1530}
    };

    public static double[][] MaxPoints = { // data max snelheid
            {60,1060},
            {70,1070},
            {80,1080},
            {90,1070},
            {100,1070},
            {110,1110},
            {120,1120},
            {130,1140},
            {140,1150},
            {150,1200},
            {160,1210},
            {170,1220},
            {180,1240},
            {190,1260},
            {200,1280},
            {210,1300},
            {220,1310},
            {230,1340},
            {240,1370},
            {250,1380},
            {260,1420},
            {270,1440},
            {280,1440},
            {290,1450},
            {300,1490},
            {310,1490}
            //{320,1520},
            //{330,1550},
            //{340,1560},
            //{350,1570},
            //{360,1570},
            //{370,1570},
            //{380,1570}
    };
    public static double P = 500.0;
    public static double D = 0.0;
    public static double F = 14.0;
    private double lP = P;
    private double lD = D;
    private double lF = F;
    public static double MinToMax = 0.2; //percentage min-max
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
                break;
            case BLUE:
                ShootTo = new Pose(11,135);
                break;
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

    public double setSpeeds(Pose RobotPos) {
        if (!PreTargeting) {
            double distance = (Math.sqrt(Math.pow(RobotPos.getY() - ShootTo.getY(), 2) + Math.pow(RobotPos.getX() - ShootTo.getX(), 2)) - 8) * 2.54;
            MaxSpeed = interpolate(MaxPoints, distance) - 5;
            MinSpeed = interpolate(MinPoints, distance) + 5;
            TargetVel = MinSpeed + (MaxSpeed - MinSpeed) * MinToMax;
            return distance;
        }
        return 0;
    }

    public void PreTargetTo(Pose RobotPos) {
        setSpeeds(RobotPos);
        PreTargeting = true;
    }

    public void Arived() {
        PreTargeting = false;
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
