/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 */

@TeleOp(name = "WindSpren Simple Teleop 2", group = "Linear OpMode")
public class SimpleTeleop extends OpMode {

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private double smoothing = 7.5;
    private double last_time = 0;

    private final InputScaler scaler = new InputScaler();
    public double lerp(double a, double b, double f)
    {
        return a * (1.0 - f) + (b * f);
    }

    static class InputScaler {
        // TODO: Tune these values for your application
        // This does NOT create any mechanical advantage, it is purely for control
        private final double[] translationGears = { 0.3, 0.6, 0.8, 1.0 };
        private final double[] rotationGears = { 0.3, 0.6, 0.8, 1.0 };

        public int gear = 1; // the index of the gear in use


        // Curves exponents
        // higher: more control at slow, less control at speed
        // lower : less control at slow, more control at speed
        // n < 1 or n > 3: not recommended
        // n = 1: linear
        // 1 < n < 3: recommended
        // TODO: experiment to find your optimal value
        private static final double translationCurveExponent = 2.0;
        private static final double rotationCurveExponent = 2.0;

        // Applies a signed exponential curve to controller input.
        // This preserves the input direction while adjusting sensitivity.
        private double applyInputCurve(double input, double exponent) {
            return Math.pow(Math.abs(input), exponent) * Math.signum(input);
        }

        // Applies both gearing and input curves to translation & rotation input
        public double getScaledInput(double translationInput, double rotationInput) {
            double curvedTranslation = applyInputCurve(translationInput, translationCurveExponent) * translationGears[gear];
            double curvedRotation = applyInputCurve(rotationInput, rotationCurveExponent) * rotationGears[gear];
            return curvedTranslation + curvedRotation;
        }
    }


    // Runs once when init has been pressed
    @Override
    public void init() {
        // TODO: to the names assigned during the robot configuration step on the Driver Station.
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "leftFront");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "leftBack");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "rightBack");

        // TODO:
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    // Runs once when start has been pressed
    @Override
    public void init_loop() {

        runtime.reset();
    }


    // Loops until the end of the match (driver presses STOP)
    @Override
    public void loop() {
        double deltaTime = runtime.seconds() - last_time;
        last_time = runtime.seconds();
        double max;

        // switch gears
        if (gamepad1.a) {
            scaler.gear = 0;
        } else if (gamepad1.b) {
            scaler.gear = 1;
        } else if (gamepad1.x) {
            scaler.gear = 2;
        } else if (gamepad1.y) {
            scaler.gear = 3;
        }

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.right_stick_x;
        double yaw     =  gamepad1.left_stick_x;

        // Combine the joystick requests for each axis-motion to determine and scale each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double dsmooth = Math.min(deltaTime * smoothing, 0.75);
        double frontLeftPower  = lerp(frontLeftDrive.getPower(), scaler.getScaledInput(axial + lateral,  yaw), dsmooth);
        double frontRightPower = lerp(frontRightDrive.getPower(), scaler.getScaledInput(axial - lateral, -yaw), dsmooth);
        double backLeftPower   = lerp(backLeftDrive.getPower(), scaler.getScaledInput(axial - lateral,  yaw), dsmooth);
        double backRightPower  = lerp(backRightDrive.getPower(), scaler.getScaledInput(axial + lateral, -yaw), dsmooth);

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        // Send calculated power to wheels
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
        telemetry.addData("smooth", dsmooth);
        telemetry.update();
    }
}
