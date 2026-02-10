package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Turret {
    public DcMotorEx turret;
    private PIDFController GoalPID = new PIDFController(new PIDFCoefficients(0,0,0,0));
    static public double kp = 1.35;
    static public double kd = 0.12;
    static public double kf = 0.1;
    private Pose Goal = new Pose(3, 141);

    public Turret(HardwareMap hwm){
        turret = hwm.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void Update(ComponentShell comps){
        GoalPID = new PIDFController(new PIDFCoefficients(kp,0,kd,kf));
        double deltaY = Goal.getY() - ((comps.limePos.getPose().getY() + 72) - Math.cos(comps.limePos.getHeading())); //TODO: field coords need to be translated from limelight to pedropathing ( i think)
        double deltaX = Goal.getX() - ((comps.limePos.getPose().getX() + 72) + Math.sin(comps.limePos.getHeading()));
        double alpha = Math.atan2(deltaY, deltaX);
        double goalAngle = alpha - Math.PI;

        double turretAngle = ticksToRadians(turret.getCurrentPosition());
        GoalPID.updatePosition(turretAngle);
        double deltaAngle = goalAngle - comps.limePos.getHeading();
        GoalPID.setTargetPosition(deltaAngle);

        turret.setPower(Math.min(Math.max(GoalPID.run(),-1),1));
    }
    private double ticksToRadians(int ticks) {
        // Depends on your gear ratio and encoder CPR
        // Example: if one full rotation = 2000 ticks
        return (ticks / 2000.0) * 2 * Math.PI;
    }

}
