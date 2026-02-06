package org.firstinspires.ftc.teamcode.drive;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.ComponentShell;
import org.firstinspires.ftc.teamcode.components.Storage;
@Configurable
@TeleOp(name="Blue Reset", group="Linear OpMode")
public class BlueReset extends LinearOpMode {
    public static Pose startingPose = new Pose(56, 9, Math.toRadians(270)); //See ExampleAuto to understand how to use this //x = 9 without triangles

    @Override
    public void runOpMode() {
        Storage.write(ComponentShell.Alliance.BLUE, startingPose);
    }
}
