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
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.RaceParallelAction;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotState;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Basebot Auto")
public class BaseBotAuto extends LinearOpMode {
    RobotHardware robot = null;
    double cycles = 4;
    boolean close;
    boolean pattern;
    Alliance alliance;
//    private final Prompter prompter = new Prompter(this);
    public double sideMultiplier;
    final double MOTOR_VELOCITY = 1145;

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

        boolean lastDpadDown = false;
        boolean lastDpadUp = false;

        boolean closeChosen = false;
        while (opModeInInit()) {
            if (alliance == null) {
                telemetry.addLine("Selected Side: Press X for Blue, B for Red");
            } else {
                telemetry.addData("Selected Side", alliance);
            }
            if (!closeChosen) {
                telemetry.addLine("Close Shot: Press A for true, Y for false");
            } else {
                telemetry.addData("Close Shot", close);
            }

            telemetry.addData("Cycles", cycles);
            telemetry.addData("pattern first", pattern);
            telemetry.update();

            if (gamepad1.x) {
                alliance = Alliance.BLUE;
            }

            if (gamepad1.b) {
                alliance = Alliance.RED;
            }
            if (gamepad1.dpad_right) {
                pattern = true;
            }
            if (gamepad1.dpad_left) {
                pattern = false;
            }

            if (gamepad1.a) {
                close = true;
                closeChosen = true;
            }

            if (gamepad1.y) {
                close = false;
                closeChosen = true;
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
            farStart = new Pose2d(new Vector2d(64.75, -7.125), Math.toRadians(180));
        } else {
            farStart = new Pose2d(new Vector2d(64.75, 9), Math.toRadians(180));
        }
        Pose2d start = close ? closeStart : farStart;

        robot = new RobotHardware(hardwareMap, start);

        robot.limelight.pipelineSwitch(0);
        LLResult result = robot.limelight.getLatestResult();
        sleep(10);
        int fiducialId = 0;
        if (result != null && result.getFiducialResults() != null
                && !result.getFiducialResults().isEmpty()) {
            fiducialId = result.getFiducialResults().get(0).getFiducialId();
        }

        if (alliance.equals(Alliance.BLUE)) {
            robot.limelight.pipelineSwitch(Pipeline.BLUE_PIPELINE.getValue());
            robot.prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
            facingGate = 270;
        } else {
            robot.limelight.pipelineSwitch(Pipeline.RED_PIPELINE.getValue());
            robot.prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
            facingGate = 90;
        }

        sideMultiplier = alliance.getValue();

        PPG = new Pose2d(new Vector2d(33, 25 * sideMultiplier), Math.toRadians(facingGate));
        PGP = new Pose2d(new Vector2d(12, 25 * sideMultiplier), Math.toRadians(facingGate));
        GPP = new Pose2d(new Vector2d(-10.5, 25 * sideMultiplier), Math.toRadians(facingGate));
        farShot = new Pose2d(new Vector2d(60, 8 * sideMultiplier), Math.toRadians(alliance.equals(Alliance.BLUE) ? 200: 155));
        closeShot = new Pose2d(new Vector2d(-10, 10 * sideMultiplier), Math.toRadians(225));
        farStart = new Pose2d(new Vector2d(64.75, 7.125 * sideMultiplier), Math.toRadians(180));
        closeStart = new Pose2d(new Vector2d(-58, 50 * sideMultiplier), Math.toRadians(45));
        Pose2d lastIntake;

        switch (fiducialId) {
            case 21:
                lastIntake = GPP;
                break;
            case 22:
                lastIntake = PGP;
                break;
            case 23:
                lastIntake = PPG;
                break;
            default:
                lastIntake = close ? GPP : PPG;
                break;
        }

        Pose2d targetShot = close ? closeShot : farShot;

        // press start
        waitForStart();
        timer.start();
        resetRuntime();

        // If zero cycles is selected, we get off the line
        if (cycles == 0) {
            Actions.runBlocking(
                    robot.actionBuilder(start)
                            .strafeToSplineHeading(close ? new Vector2d(-50, 20 * sideMultiplier) : new Vector2d(42, 7.125 * sideMultiplier), facingGate)
                            .build()
            );
            RobotState.setCurrentPose(robot.localizer.getPose());
            robot.prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_7);
            stop();
            return;
        }

        // choose which start to use based on the config segment
        Actions.runBlocking(
                robot.actionBuilder(start)
                        .strafeToSplineHeading(targetShot.position, targetShot.heading)
                        .build()
        );
        RobotState.setCurrentPose(robot.localizer.getPose());


        // shoot and autoalign
        Actions.runBlocking(new SequentialAction(
                new RaceParallelAction(
                        robot.spinUpShooter(MOTOR_VELOCITY),
                        robot.autoalign()
                ),
                robot.shootBall(3, MOTOR_VELOCITY)
        ));
        telemetry.addData("Balls", RobotState.getBallsIn());
        telemetry.update();

        if (cycles == 1) {
            RobotState.setCurrentPose(robot.localizer.getPose());
            robot.prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_2);
            RobotState.setCurrentPose(robot.localizer.getPose());
            stop();
            return;
        }

        Actions.runBlocking(
                robot.actionBuilder(robot.localizer.getPose())
                        .strafeToSplineHeading(
                            PPG.position,
                            PPG.heading
                        )
                        .build()
        );
        RobotState.setCurrentPose(robot.localizer.getPose());


        Actions.runBlocking(
            new SequentialAction(
                robot.actionBuilder(robot.localizer.getPose())
                    .strafeToSplineHeading(
                            PPG.position,
                            PPG.heading
                    ).build(),
                robot.startIntake(),
                new RaceParallelAction(
                    robot.actionBuilder(robot.localizer.getPose())
                        .strafeToSplineHeading(
                                new Vector2d(PPG.position.x, (sideMultiplier * RobotState.getY(RobotState.getBallsIn()))),
                                Math.toRadians(facingGate),
                                new TranslationalVelConstraint(32.0),
                                new ProfileAccelConstraint(-15.0, 50.0)
                        )
                    .build(),
                    robot.intakeTracker.trackIntakeAction()
                ),
                robot.stopIntake(),
                robot.actionBuilder(robot.localizer.getPose())
                    .strafeToSplineHeading(targetShot.position, targetShot.heading)
                    .build()
            )
        );
        Actions.runBlocking(
                new SequentialAction(
                        new RaceParallelAction(
                                robot.spinUpShooter(MOTOR_VELOCITY),
                                robot.autoalign()
                        ),
                        robot.shootBall(3, MOTOR_VELOCITY)
                )
        );
        RobotState.setCurrentPose(robot.localizer.getPose());


        telemetry.addData("Balls", RobotState.getBallsIn());
        telemetry.update();

        if (cycles == 2) {
            robot.prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_7);
            RobotState.setCurrentPose(robot.localizer.getPose());
            stop();
            return;
        }

        // go to another position (Intake segment 3)
        Actions.runBlocking(
                robot.actionBuilder(robot.localizer.getPose())
                        .strafeToLinearHeading(
                                PGP.position,
                                PGP.heading
//                                new TranslationalVelConstraint(15.0),
//                                new ProfileAccelConstraint(-15.0, 40.0)
                        )
                        .build()
        );
        RobotState.setCurrentPose(robot.localizer.getPose());

        // intake for how many balls we have (Intake segment 4)
        Actions.runBlocking(
                new SequentialAction(
                        robot.startIntake(),
                        new RaceParallelAction(
                                robot.actionBuilder(robot.localizer.getPose())
                                        .strafeToSplineHeading(
                                                new Vector2d(PGP.position.x, (sideMultiplier * RobotState.getY(RobotState.getBallsIn()))),
                                                Math.toRadians(facingGate),
                                                new TranslationalVelConstraint(15.0),
                                                new ProfileAccelConstraint(-15.0, 45.0)
                                        )
                                        .build(),
                                robot.intakeTracker.trackIntakeAction()
                        ),
                        robot.stopIntake()
                )
        );
        RobotState.setCurrentPose(robot.localizer.getPose());

        Actions.runBlocking(
                robot.actionBuilder(robot.localizer.getPose())
                        .strafeToSplineHeading(targetShot.position, targetShot.heading)
                        .build()
        );
        RobotState.setCurrentPose(robot.localizer.getPose());


        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                            robot.spinUpShooter(MOTOR_VELOCITY),
                            robot.autoalign()
                        ),
                        robot.shootBall(3, MOTOR_VELOCITY)
                )
        );
        RobotState.setCurrentPose(robot.localizer.getPose());

        telemetry.addData("Balls", RobotState.getBallsIn());
        telemetry.update();

        if (cycles == 3) {
            robot.prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_2);
            stop();
            return;
        }

        // open the gate and redo
        Actions.runBlocking(
                robot.actionBuilder(robot.localizer.getPose())
                        .strafeToSplineHeading(new Vector2d(2, 58 * sideMultiplier), Math.toRadians(90))
                        .build()
        );
        RobotState.setCurrentPose(robot.localizer.getPose());

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
                        new RaceParallelAction(
                                robot.actionBuilder(robot.localizer.getPose())
                                        .strafeToSplineHeading(
                                                new Vector2d(GPP.position.x, (sideMultiplier * RobotState.getY(RobotState.getBallsIn()))),
                                                Math.toRadians(facingGate),
                                                new TranslationalVelConstraint(15.0),
                                                new ProfileAccelConstraint(-15.0, 40.0)
                                        )
                                        .build(),
                                robot.intakeTracker.trackIntakeAction()
                        ),
                        robot.stopIntake()
                )
        );
        RobotState.setCurrentPose(robot.localizer.getPose());
        telemetry.addData("Balls", RobotState.getBallsIn());
        telemetry.update();

        Actions.runBlocking(
                robot.actionBuilder(robot.localizer.getPose())
                        .strafeToSplineHeading(closeShot.position, Math.toRadians(225))
                        .build()
        );
        RobotState.setCurrentPose(robot.localizer.getPose());

        Actions.runBlocking(
            new SequentialAction(
                new ParallelAction(
                        robot.spinUpShooter(1100),
                        robot.autoalign()
                ),
                robot.shootBall(3, 1100)
            )
        );
        RobotState.setCurrentPose(robot.localizer.getPose());
        telemetry.addData("Balls", RobotState.getBallsIn());
        telemetry.update();
    }
}
