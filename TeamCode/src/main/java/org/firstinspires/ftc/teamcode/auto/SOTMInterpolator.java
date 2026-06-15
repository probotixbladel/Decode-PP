package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathPoint;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.components.ComponentShell;

public class SOTMInterpolator implements HeadingInterpolator {
    private Follower follower;
    private ComponentShell comps;
    public static double offsetX = 1.5748;

    @Override
    public double interpolate(PathPoint closestPoint) {
        double dy = comps.shooter.ShootTo.getY() - (follower.getPose().getY() - Math.cos(follower.getHeading()) * offsetX);
        double dx = comps.shooter.ShootTo.getX() - (follower.getPose().getX() + Math.sin(follower.getHeading()) * offsetX);
        double alpha = Math.atan2(dy, dx);
        double beta = alpha - Math.PI;
        return beta;
    }

    public SOTMInterpolator giveInfo(Follower follower, ComponentShell comps) {
        this.follower = follower;
        this.comps = comps;
        return this;
    }
}