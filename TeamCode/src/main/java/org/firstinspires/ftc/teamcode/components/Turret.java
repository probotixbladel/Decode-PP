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
    static private double deadZone1 = 100;
    static private double deadZone2 = 80;
    private double dzLow;
    private double dzHigh;
    private Pose Goal = new Pose(3, 141);
    private double startRot = 90;

    public Turret(HardwareMap hwm){
        turret = hwm.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        GoalPID = new PIDFController(new PIDFCoefficients(kp,0,kd,kf));
    }

    public void Update(ComponentShell comps) {
        double deltaY = Goal.getY() - comps.follower.getPose().getY();
        double deltaX = Goal.getX() - comps.follower.getPose().getX();
        double alpha = Math.atan2(deltaY, deltaX);
        //double goalAngle = Math.toDegrees(-alpha);
        double goalAngle = 45;
        if(goalAngle < 0){
            goalAngle += 360;
        }

        dzLow = Math.min(deadZone1, deadZone2);
        dzHigh = 360 - Math.max(deadZone1, deadZone2);

        // Normalize both into dead-zone space
        double normalizedGoal   = NormalizeAngleToDeadZone(goalAngle);
        double normalizedTurret = NormalizeAngleToDeadZone(
                Math.toDegrees(ticksToRadians(turret.getCurrentPosition()) + comps.follower.getHeading()) + startRot
        );

        // Wrap goal into [0, dzHigh] — if it's outside, snap to nearest boundary
        if(normalizedGoal > dzHigh){
            if(Math.abs(normalizedGoal - 360) < Math.abs(normalizedGoal - dzHigh)){
                normalizedGoal = 0;
            }
            else{
                normalizedGoal = dzHigh;
            }
        }

        GoalPID.updatePosition(Math.toRadians(normalizedTurret));
        GoalPID.setTargetPosition(Math.toRadians(normalizedGoal));

        turret.setPower(Math.min(Math.max(GoalPID.run(), -1), 1));
    }
        private double ticksToRadians(int ticks) {
        // Depends on your gear ratio and encoder CPR
        // if one full rotation = 145.6 ticks tDriving/tDriven;
        return ((ticks / 145.6) * 2 * Math.PI) * tDriving/tDriven;
    }

    private double NormalizeAngleToDeadZone(double angle) {
        double shifted = angle - dzLow;
        if(shifted < 0){
            shifted += 360;
        }
        return shifted;
    }

}
