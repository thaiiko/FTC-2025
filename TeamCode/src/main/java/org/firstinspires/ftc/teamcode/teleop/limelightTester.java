package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp
public class limelightTester extends LinearOpMode {
    RobotHardware robot;
    int selectedPipeline;

    public void runOpMode() {
        RobotHardware robot = new RobotHardware(hardwareMap, new Pose2d(0,0,0));

        boolean lastDpadUp = false;
        boolean lastDpadDown = false;
        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.dpad_up && !lastDpadUp) {
                selectedPipeline++;
            }
            if (gamepad1.dpad_down && !lastDpadDown) {
                selectedPipeline--;
            }
            lastDpadDown = gamepad1.dpad_down;
            lastDpadUp = gamepad1.dpad_up;

            robot.limelight.pipelineSwitch(selectedPipeline);
            LLResult result = robot.limelight.getLatestResult();

            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("ta", result.getTa());
            telemetry.addData("selected pipeline", selectedPipeline);
            telemetry.update();
        }
    }
}
