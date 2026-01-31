package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp
public class FlywheelTuner extends OpMode {
    RobotHardware robot;
    public double highVelocity = 1500;
    public double lowVelocity = 1500;

    public double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 1;

    public void init() {
        robot = new RobotHardware(hardwareMap, new Pose2d(0, 0, 0));
        robot.lShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        robot.lShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        robot.rShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        if (gamepad1.b) { stepIndex = (stepIndex + 1) % stepSizes.length; }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        } else if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        } else if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        robot.lShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        robot.rShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        robot.lShooter.setVelocity(curTargetVelocity);

        double curVelocity = robot.lShooter.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", curVelocity);
        telemetry.addData("Error", error);
        telemetry.addData("P", P);
        telemetry.addData("F", F);
        telemetry.addData("Step Size", stepSizes[stepIndex]);
        telemetry.update();
    }
}
