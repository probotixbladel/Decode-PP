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
    static public double kp = 1.8;
    static public double kd = 0.1;
    static public double kf = 10;
    //TODO: tune deadzones!
    static private double deadZone1 = 85;
    static private double deadZone2 = 75;
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

        double dzLow  = Math.toRadians(Math.min(deadZone1, deadZone2)) + comps.follower.getHeading() + startRot;
        double dzHigh = Math.toRadians(Math.max(deadZone1, deadZone2)) + comps.follower.getHeading() + startRot;

        goalAngle = turretAngle + Math.IEEEremainder(goalAngle - turretAngle, 2 * Math.PI);
        if(goalAngle >= dzLow && goalAngle <= dzHigh){
            double distToLow = Math.abs(goalAngle - dzLow);
            double distToHigh = Math.abs(goalAngle - dzHigh);
            goalAngle = (distToLow < distToHigh) ? dzLow : dzHigh;
        }
        if (turretAngle >= dzLow && turretAngle <= dzHigh){
            double distToLow = Math.abs(turretAngle - dzLow);
            double disToHigh = Math.abs(turretAngle - dzHigh);
            if(distToLow < disToHigh && goalAngle >= dzHigh) {
                //CW
                goalAngle -= 2 * Math.PI;
            }else if(goalAngle <= dzLow){
                //CCW
                goalAngle += 2 * Math.PI;
            }
        }else if ((turretAngle <= dzLow && goalAngle >= dzHigh)
                || (turretAngle >= dzHigh && goalAngle <= dzLow)){
            // Path crosses deadzone, go the long way around
            if (goalAngle > turretAngle) {
                //CW
                goalAngle -= 2 * Math.PI;
            } else {
                //CCW
                goalAngle += 2 * Math.PI;
            }
        }

        GoalPID.updatePosition(turretAngle);
        GoalPID.setTargetPosition(goalAngle);

        turret.setPower(Math.min(Math.max(GoalPID.run(), -1), 1));
    }
    private double ticksToRadians(int ticks) {
        // Depends on your gear ratio and encoder CPR
        // if one full rotation = 145.6 ticks tDriving/tDriven;
        return ((ticks / 145.6) * 2 * Math.PI) * tDriving/tDriven;
    }

}
