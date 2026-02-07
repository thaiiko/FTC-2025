package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp
public class LightTester extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(hardwareMap, new Pose2d(0,0, 0));
        while (!isStopRequested()) {
            telemetry.addLine("Light 1: X");
            telemetry.addLine("Light 2: Y");
            telemetry.addLine("Light 3: A");
            telemetry.addLine("Light 4: B");

            if (gamepad1.x) {
                robot.light1.setPosition(0.7);
            }
            if (gamepad1.y ) {
                robot.light2.setPosition(0.7);
            }
            if (gamepad1.a) {
                robot.light3.setPosition(0.7);
            }
            if (gamepad1.b) {
                robot.light4.setPosition(0.7);
            }

            telemetry.update();
        }
    }
}
