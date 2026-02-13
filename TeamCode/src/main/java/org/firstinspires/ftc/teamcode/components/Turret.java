package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Turret {
    public DcMotorEx turret;
    private PIDFController GoalPID;
    public int tDriven = 130;
    public int tDriving = 10;
    public double motorRatio = (1+(46/11));
    static public double kp = 1.8;
    static public double kd = 0.1;
    static public double kf = 10;
    private Pose Goal = new Pose(3, 141);
    private double startRot = 0.5 * Math.PI;

    public Turret(HardwareMap hwm){
        turret = hwm.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        GoalPID = new PIDFController(new PIDFCoefficients(kp,0,kd,kf));
    }

    public void Update(ComponentShell comps){
        double deltaY = Goal.getY() - comps.follower.getPose().getY();
        double deltaX = Goal.getX() - comps.follower.getPose().getX();
        double alpha = Math.atan2(deltaY, deltaX);
        double goalAngle = alpha - 0.5 * Math.PI;

        double turretAngle = ticksToRadians(turret.getCurrentPosition()) + comps.follower.getHeading() + startRot;
        if(turretAngle < 0){
            turretAngle += 360;
        }
        GoalPID.updatePosition(turretAngle);
        GoalPID.setTargetPosition(goalAngle);

        turret.setPower(Math.min(Math.max(GoalPID.run(),-1),1));
    }
    private double ticksToRadians(int ticks) {
        // Depends on your gear ratio and encoder CPR
        // if one full rotation = 145.6 ticks tDriving/tDriven;
        return ((ticks / 145.6) * 2 * Math.PI) * tDriving/tDriven;
    }

}
