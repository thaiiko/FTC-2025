package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Pipeline;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotState;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Basebot Auto")
public class BaseBotAuto extends LinearOpMode {
    RobotHardware robot = null;
    double cycles = 4;
    boolean close;
    Alliance alliance;
//    private final Prompter prompter = new Prompter(this);
    public double sideMultiplier;

    double facingGate;
    Pose2d farShot;
    Pose2d closeShot;
    Pose2d farStart;
    Pose2d closeStart;
    Pose2d PPG;
    Pose2d PGP;
    Pose2d GPP;

    @Override
    public void runOpMode() {
        TimerEx timer = new TimerEx(30, TimeUnit.SECONDS);
        robot = new RobotHardware(hardwareMap, new Pose2d(64.75, -7.125, Math.toRadians(180)));

        robot.limelight.pipelineSwitch(1);
        boolean lastDpadDown = false;
        boolean lastDpadUp = false;

        boolean closeChoosen = false;
        while (opModeInInit()) {
            if (alliance == null) {
                telemetry.addLine("Selected Side: Press X for Blue, B for Red");
            } else {
                telemetry.addData("Selected Side", alliance);
            }
            if (!closeChoosen) {
                telemetry.addLine("Close Shot: Press A for true, B for false");
            } else {
                telemetry.addData("Close Shot", close);
            }
            telemetry.addData("Cycles", cycles);
            telemetry.update();

            if (gamepad1.x) {
                alliance = Alliance.BLUE;
            }

            if (gamepad1.b) {
                alliance = Alliance.RED;
            }

            if (gamepad1.a) {
                close = true;
                closeChoosen = true;
            }

            if (gamepad1.y) {
                close = false;
                closeChoosen = true;
            }

            if (gamepad1.dpad_down && !lastDpadDown) {
                cycles = Math.max(cycles - 1, 0);
            }

            if (gamepad1.dpad_up && !lastDpadUp) {
                cycles = Math.min(cycles + 1, 4);
            }

            lastDpadDown = gamepad1.dpad_down;
            lastDpadUp = gamepad1.dpad_up;


        }
        if (alliance.equals(Alliance.BLUE)) {
            robot.limelight.pipelineSwitch(Pipeline.BLUE_PIPELINE.getValue());
        } else {
            robot.limelight.pipelineSwitch(Pipeline.RED_PIPELINE.getValue());
        }


        sideMultiplier = alliance.getValue();

        if (sideMultiplier == -1) {
            facingGate = 269;
        } else if (sideMultiplier == 1) {
            facingGate = 89;
        }

        PPG = new Pose2d(new Vector2d(40, 29 * sideMultiplier), Math.toRadians(facingGate));
        PGP = new Pose2d(new Vector2d(13, 29 * sideMultiplier), Math.toRadians(facingGate));
        GPP = new Pose2d(new Vector2d(-10.5, 29 * sideMultiplier), Math.toRadians(facingGate));
        farShot = new Pose2d(new Vector2d(60, 7.125 * sideMultiplier), Math.toRadians(200));
        closeShot = new Pose2d(new Vector2d(-10, 10 * sideMultiplier), Math.toRadians(225));
        farStart = new Pose2d(new Vector2d(64.75, 7.125 * sideMultiplier), Math.toRadians(180));
        closeStart = new Pose2d(new Vector2d(-50, 50 * sideMultiplier), Math.toRadians(135));

        Pose2d targetShot = close ? closeShot : farShot;
        Pose2d lastIntake;

        LLResult result = robot.limelight.getLatestResult();
        int fiducialId = 0;
        if (result.getFiducialResults().get(0) != null) {
            fiducialId = result.getFiducialResults().get(0).getFiducialId();
        }

        switch (fiducialId) {
            case 21:
                lastIntake = GPP;
            case 22:
                lastIntake = PGP;
            case 23:
                lastIntake = PPG;
            default:
                lastIntake = close ? GPP : PPG;
        }

        // press start
        waitForStart();
        timer.start();
        resetRuntime();

        // If zero cycles is selected, we get off the line
        if (cycles == 0) {
            Actions.runBlocking(
                    robot.actionBuilder(close ? closeStart : farStart)
                            .strafeToSplineHeading(new Vector2d(42, 7.125 * sideMultiplier), Math.toRadians(180))
                            .build()
            );
            stop();
            return;
        }

        // pre speed up the shooter
        robot.lShooter.setVelocity(1100);
        robot.rShooter.setVelocity(1100);

        // choose which start to use based on the config segment
        Actions.runBlocking(
                robot.actionBuilder(close ? closeStart : farStart)
                        .strafeToSplineHeading(targetShot.position, targetShot.heading)
                        .build()
        );


        // shoot and autoalign
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        robot.spinUpShooter(1100),
                        robot.autoalign()
                ),
                robot.shootBall(3, 1100)
        ));
        telemetry.addData("Balls", RobotState.getBallsIn());
        telemetry.update();

        if (cycles == 1) {
            stop();
            return;
        }

        // go to PPG (Intake segment 1)
        Actions.runBlocking(
                robot.actionBuilder(robot.localizer.getPose())
                        .strafeToSplineHeading(
                                PPG.position,
                                PPG.heading,
                                new TranslationalVelConstraint(15.0),
                                new ProfileAccelConstraint(-15.0, 40.0)
                        )
                        .build()
        );

        Actions.runBlocking(
                robot.startIntake()
        );

        // Intake until we take in the amount of balls we need (Intake segment 2)
        Actions.runBlocking(
                robot.actionBuilder(robot.localizer.getPose())
                        .strafeToSplineHeading(
                                new Vector2d(PPG.position.x, RobotState.getY(RobotState.getBallsIn())),
                                Math.toRadians(269),
                                new TranslationalVelConstraint(15.0),
                                new ProfileAccelConstraint(-15.0, 40.0)
                        )
                        .build()
        );

        // run back to the back shooting area.
        Actions.runBlocking(
                robot.actionBuilder(robot.localizer.getPose())
                        .strafeToSplineHeading(targetShot.position, targetShot.heading)
                        .build()
        );

        // shoot #2
        Actions.runBlocking(
                new SequentialAction(
                        robot.startIntake(),
                        robot.shootBall(3, 1100)
                )
        );

        telemetry.addData("Balls", RobotState.getBallsIn());
        telemetry.update();

        if (cycles == 2) {
            stop();
            return;
        }

        // go to another position (Intake segment 3)
        Actions.runBlocking(
                robot.actionBuilder(robot.localizer.getPose())
                        .strafeToLinearHeading(
                                PGP.position,
                                PGP.heading,
                                new TranslationalVelConstraint(15.0),
                                new ProfileAccelConstraint(-15.0, 40.0)
                        )
                        .build()
        );

        // intake for how many balls we have (Intake segment 4)
        Actions.runBlocking(
                robot.actionBuilder(robot.localizer.getPose())
                        .strafeToSplineHeading(
                                new Vector2d(PGP.position.x, RobotState.getY(RobotState.getBallsIn())),
                                Math.toRadians(269),
                                new TranslationalVelConstraint(15.0),
                                new ProfileAccelConstraint(-15.0, 45.0)
                        )
                        .build()
        );

        Actions.runBlocking(
                robot.actionBuilder(robot.localizer.getPose())
                        .strafeToSplineHeading(targetShot.position, targetShot.heading)
                        .build()
        );


        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                            robot.startIntake(),
                            robot.autoalign()
                        ),
                        robot.shootBall(3, 1100)
                )
        );

        telemetry.addData("Balls", RobotState.getBallsIn());
        telemetry.update();

        if (cycles == 3) {
            stop();
            return;
        }

        // open the gate and redo
        Actions.runBlocking(
                robot.actionBuilder(robot.localizer.getPose())
                        .strafeToSplineHeading(new Vector2d(2, -58), Math.toRadians(90))
                        .build()
        );

/*
        Actions.runBlocking(
                robot.actionBuilder(robot.localizer.getPose())
                        .strafeToSplineHeading(lastIntake.position, Math.toRadians(269))
                        .build()
        );
*/

        Actions.runBlocking(
                new SequentialAction(
                        robot.actionBuilder(robot.localizer.getPose())
                                .strafeToSplineHeading(
                                        GPP.position,
                                        GPP.heading,
                                        new TranslationalVelConstraint(15.0),
                                        new ProfileAccelConstraint(-15.0, 40.0)
                                )
                                .build(),
                        robot.startIntake(),
                        robot.actionBuilder(robot.localizer.getPose())
                                .strafeToSplineHeading(
                                        new Vector2d(GPP.position.x, RobotState.getY(RobotState.getBallsIn())),
                                        Math.toRadians(269),
                                        new TranslationalVelConstraint(15.0),
                                        new ProfileAccelConstraint(-15.0, 40.0)
                                )
                                .build(),
                        robot.stopIntake()
                )
        );
        telemetry.addData("Balls", RobotState.getBallsIn());
        telemetry.update();

        Actions.runBlocking(
                robot.actionBuilder(robot.localizer.getPose())
                        .strafeToSplineHeading(closeShot.position, Math.toRadians(225))
                        .build()
        );

        Actions.runBlocking(
            new SequentialAction(
                new ParallelAction(
                        robot.autoalign(),
                        robot.spinUpShooter(1100)
                ),
                robot.shootBall(3, 1100)
            )
        );
        telemetry.addData("Balls", RobotState.getBallsIn());
        telemetry.update();


    }
}