package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;
import java.util.Vector;

@Autonomous(name = "Basebot Auto")
public class BaseBotAuto extends LinearOpMode {
    RobotHardware robot = null;
    PoseVelocity2d pose;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap, new Pose2d(64.75, -7.125, Math.toRadians(180)));

        robot.limelight.pipelineSwitch(1);

        waitForStart();
        resetRuntime();

        robot.lShooter.setVelocity(1100);
        robot.rShooter.setVelocity(1100);

//        robot.lShooter.setPower(0.60);
//        robot.rShooter.setPower(0.60);

        Actions.runBlocking(
                robot.actionBuilder(new Pose2d(64.75, -7.125, Math.toRadians(180)))
                        .strafeToSplineHeading(new Vector2d(60, -7.125), Math.toRadians(200))
                        .build()
        );


//        LLResult result = robot.limelight.getLatestResult();
//        while (result.getTx() > 0.5 || result.getTx() < -0.5) {
//            result = robot.limelight.getLatestResult();
//            robot.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), result.getTx()/27.25 * -1));
//            robot.updatePoseEstimate();
//            robot.light1.setPosition(.5);
//        }
//        robot.light1.setPosition(0);

        // slightly adjust these TODO
        int balls = 3;
        boolean lastSeenBall = false;
//        double start = getRuntime();
        double lastBallTime = getRuntime();
        while (balls > 0) {
            boolean ballin = robot.distance1.getState();
            telemetry.addData("ballin", ballin);
            telemetry.addData("balls", balls);

            if (ballin && !lastSeenBall) {
                balls = balls - 1;
                lastSeenBall = true;
            } else {
                lastSeenBall = false;
            }

            robot.lShooter.setVelocity(1100);
            robot.rShooter.setVelocity(1100);

            if ((robot.lShooter.getVelocity() < 1140 && robot.lShooter.getVelocity() > 1000) &&
                    (robot.rShooter.getVelocity() < 1170 && robot.rShooter.getVelocity() > 1000)) {
                robot.light1.setPosition(0.7);
                robot.intake.setPower(0.7);
                robot.index.setPower(1);
            } else {
                robot.light1.setPosition(0);
                robot.intake.setPower(0.8);
                robot.index.setPower(0);
            }
            telemetry.update();
        }
        sleep(250);

        robot.intake.setPower(1);
        Actions.runBlocking(
                robot.actionBuilder(new Pose2d(new Vector2d(60, -7.125), Math.toRadians(200)))
                        .strafeToSplineHeading(new Vector2d(37, -24), Math.toRadians(265))
                        .build()
        );
        Actions.runBlocking(
                robot.actionBuilder(new Pose2d(new Vector2d(37, -24), Math.toRadians(265)))
                        .lineToY(-30, )
                        .lineToY(-30)
                        .build()
        );
        // TODO Implement Intake

    }
}