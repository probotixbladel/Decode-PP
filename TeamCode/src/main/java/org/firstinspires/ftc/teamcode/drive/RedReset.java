package org.firstinspires.ftc.teamcode.drive;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.ComponentShell;
import org.firstinspires.ftc.teamcode.components.Storage;

@TeleOp(name="Red Reset", group="Linear OpMode")
public class RedReset extends LinearOpMode {
    public static Pose startingPose = new Pose(87.0, 9, Math.toRadians(270)); //See ExampleAuto to understand how to use this

    @Override
    public void runOpMode() {
        Storage.write(ComponentShell.Alliance.RED, startingPose);
    }
}
