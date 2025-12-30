package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.0)
            .forwardZeroPowerAcceleration(-39.07258408121004)
            .lateralZeroPowerAcceleration(-72.59243686114439)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.08,
                    0,
                    0,
                    0.025
            ))
            .translationalPIDFSwitch(4)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.0,
                    0,
                    0.0,
                    0.0
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1,
                    0,
                    0,
                    0.01
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    0.0,
                    0.0,
                    0.0,
                    0.0
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.003,
                    0,
                    0.0000,
                    0.0,
                    0.00
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0
            ))
            .drivePIDFSwitch(15)
            .centripetalScaling(0.005);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftBack")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true)
            .xVelocity(74.27791229007752)
            .yVelocity(62.529);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .hardwareMapName("pinpoint")
            .forwardPodY(-6.614173)
            .strafePodX(0.8267717)
            .distanceUnit(DistanceUnit.INCH)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    /**
     These are the PathConstraints in order:
     tValueConstraint, velocityConstraint, translationalConstraint, headingConstraint, timeoutConstraint,
     brakingStrength, BEZIER_CURVE_SEARCH_LIMIT, brakingStart

     The BEZIER_CURVE_SEARCH_LIMIT should typically be left at 10 and shouldn't be changed.
     */

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            50,
            1,
            10,
            0.9
    );

    //Add custom localizers or drivetrains here
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}