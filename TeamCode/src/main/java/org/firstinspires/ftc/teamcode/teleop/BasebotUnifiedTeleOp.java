package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Pipeline;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.Speedometer;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotState;

import java.util.List;

@TeleOp(name = "BaseBot Unified Teleop")
public class BasebotUnifiedTeleOp extends LinearOpMode {
    private ElapsedTime elapsedTime;
    private static final int INTAKE_ON_RUMBLE_MS = 220;
    private static final int INTAKE_OFF_RUMBLE_MS = 120;
    private static final int SHOOTER_IN_RANGE_RUMBLE_MS = 180;
    private static final int SHOOTER_LONG_RUN_RUMBLE_MS = 300;
    private static final double SHOOTER_READY_WINDOW_TICKS = 20.0;
    private static final double SHOOTER_LONG_RUN_SECONDS = 10.0;
    private static final double SHOOTER_IN_RANGE_RUMBLE_INTERVAL_SECONDS = 0.15;

    private volatile LLResult latestResult;
    private Thread visionThread;

    private double targetDistance;
    private double velocity;
    private double tx;
    private double ty;
    private double ta;

    private RobotHardware robot;
    private Limelight3A limelight;

    private boolean isDualMode;
    private boolean humanPlayer;
    private boolean debugMode;

    private Gamepad getMechanismGamepad() {
        return isDualMode ? gamepad2 : gamepad1;
    }

    private void addDebugTelemetry(String caption, Object value) {
        if (debugMode) {
            telemetry.addData(caption, value);
        }
    }

    @Override
    public void runOpMode() {
        double shooterRpm = Constants.Shooter.MANUAL_DEFAULT_RPM;

        elapsedTime = new ElapsedTime();
        boolean previousDpadUp = false;
        boolean previousDpadDown = false;
        boolean shooterOn = false;
        boolean bToggle = false;
        boolean xToggle = false;
        boolean intakeToggleOn = false;
        boolean shooterLongRunRumbleSent = false;
        boolean lastB = false;
        boolean lastX = false;
        boolean lastIntakeToggleButton = false;
        double shooterStartTimeSec = -1.0;
        double lastShooterInRangeRumbleTimeSec = -1.0;

        robot = new RobotHardware(hardwareMap, RobotState.getCurrentPose());
        limelight = robot.limelight;
        Speedometer speedometer = new Speedometer(robot);
        robot.prism.setStripLength(Constants.TeleOp.PRISM_STRIP_LENGTH);

        Pipeline selectedPipeline = selectModeAndPipeline();

        telemetry.addData("Controller Mode", isDualMode ? "Dual Driver" : "Single Driver");
        telemetry.addData("Selected Pipeline", selectedPipeline.name());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        limelight.pipelineSwitch(selectedPipeline.getValue());
        elapsedTime.reset();
        waitForStart();

        startVisionThread();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Velocity", robot.lShooter.getVelocity());

            fetchVariablesFromLimelight(latestResult);
            if (latestResult != null && latestResult.getFiducialResults() != null) {
                List<LLResultTypes.FiducialResult> fiducialResults = latestResult.getFiducialResults();
                for (int i = 0; i < fiducialResults.size(); i++) {
                    telemetry.addData("Tag ID " + i, fiducialResults.get(i).getFiducialId());
                }
            }

            double angleToGoalDegrees = Constants.Vision.LIMELIGHT_MOUNT_ANGLE_DEG + ty;
            double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
            double distanceToTowerInches = (Constants.Vision.GOAL_HEIGHT_IN - Constants.Vision.LIMELIGHT_HEIGHT_IN)
                    / Math.tan(angleToGoalRadians);
            speedometer.speedAnim(-gamepad1.left_stick_y);

            targetDistance = (distanceToTowerInches / 12.0) + Constants.Vision.DISTANCE_OFFSET_FT;

            double numerator = Constants.Shooter.GRAVITY_FT_S2 * Math.pow(targetDistance, 2);
            double denominator = 2.0 * Math.cos(Constants.Shooter.LAUNCH_ANGLE_RAD) * Math.cos(Constants.Shooter.LAUNCH_ANGLE_RAD)
                    * (targetDistance * Math.tan(Constants.Shooter.LAUNCH_ANGLE_RAD) - Constants.Shooter.TARGET_FEET);

            if (targetDistance <= 0 || denominator <= 0) {
                addDebugTelemetry("Invalid shot geometry (too high/far for angle): using default", null);
                velocity = Constants.Shooter.DEFAULT_FALLBACK_VELOCITY_FT_S;
            } else {
                velocity = Math.sqrt(numerator / denominator);
            }

            double motorRpm = (Constants.Shooter.RPM_MAGIC_CONSTANT * velocity)
                    / (Math.PI * Constants.Shooter.SHOOTER_WHEEL_DIAMETER_FT)
                    * Constants.Shooter.RPM_EMPIRICAL_FACTOR;
            double motorVelocity = motorRpm * Constants.Shooter.TICKS_PER_REV / 60.0;

            addDebugTelemetry("Required Launch Velocity (ft/s)", velocity);
            addDebugTelemetry("Required Motor RPM (ft/s)", motorRpm);
            addDebugTelemetry("Target Distance (feet)", targetDistance);
            addDebugTelemetry("Required Motor Velocity (ticks/s)", motorVelocity);

            double rawTurn = applyDeadband(gamepad1.right_stick_x, Constants.Drive.DEAD_BAND);
            double rawY = applyDeadband(gamepad1.left_stick_y, Constants.Drive.DEAD_BAND);
            double rawX = applyDeadband(gamepad1.left_stick_x, Constants.Drive.DEAD_BAND);

            double turn = -rawTurn * Constants.Drive.ROTATION_SCALE;
            if (bToggle) {
                turn += -((tx / Constants.Drive.LIMELIGHT_TURN_DIVISOR) * Constants.Drive.ROTATION_SCALE);
            }

            if (!(tx == 0.0 && ty == 0.0 && ta == 0.0)) {
                telemetry.addData("Target X (tx)", tx);
                telemetry.addData("Target Y (ty)", ty);
                telemetry.addData("Target Area (ta)", ta);
            }

            robot.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -Constants.Drive.TRANSLATION_GAIN * (Constants.Drive.TRANSLATION_SCALE * Math.pow(rawY, 3)),
                            -Constants.Drive.TRANSLATION_GAIN * (Constants.Drive.TRANSLATION_SCALE * Math.pow(rawX, 3))
                    ),
                    turn
            ));
            robot.updatePoseEstimate();

            Gamepad mechGP = getMechanismGamepad();
            double indexPower = 0.0;
            double intakePower = 0.0;

            if (mechGP.dpad_left && !lastIntakeToggleButton) {
                intakeToggleOn = !intakeToggleOn;
                if (intakeToggleOn) {
                    rumbleMechanismGamepad(INTAKE_ON_RUMBLE_MS);
                } else {
                    rumbleMechanismGamepad(INTAKE_OFF_RUMBLE_MS);
                }
            }

            if (mechGP.right_trigger > Constants.TeleOp.RIGHT_TRIGGER_THRESHOLD) {
                indexPower = shooterOn ? 1.0 : 0.2;
                intakePower = 0.6;
            } else if (mechGP.dpad_right) {
                intakePower = -0.8;
            } else if (intakeToggleOn && !shooterOn) {
                intakePower = 0.8;
            } else if (mechGP.y) {
                indexPower = humanPlayer ? -1.0 : -0.5;
            }


            robot.index.setPower(indexPower);
            robot.intake.setPower(intakePower);

            if (intakePower > 0.0) {
                robot.intakeTracker.trackIntake();
            }
            robot.intakeTracker.trackShoot(shooterOn);

            if (mechGP.dpad_up && !previousDpadUp) {
                shooterRpm += Constants.Shooter.RPM_STEP;
            }
            if (mechGP.dpad_down && !previousDpadDown) {
                shooterRpm = Math.max(0.0, shooterRpm - Constants.Shooter.RPM_STEP);
            }

            telemetry.addData("Manual Shooter RPM", shooterRpm);

            if (mechGP.right_bumper) {
                shooterOn = true;
            } else if (mechGP.left_bumper) {
                shooterOn = false;
            }

            if (mechGP.x && !lastX) {
                xToggle = !xToggle;
            }
            if (gamepad1.b && !lastB) {
                bToggle = !bToggle;
            }

            if (shooterOn) {
                if (!xToggle) {
                    robot.setShooterVelocity(shooterRpm);
                } else {
                    robot.setShooterVelocity(motorVelocity);
                }

                if (shooterStartTimeSec < 0.0) {
                    shooterStartTimeSec = getRuntime();
                    shooterLongRunRumbleSent = false;
                    lastShooterInRangeRumbleTimeSec = -1.0;
                }

                double targetShooterVelocity = xToggle ? motorVelocity : shooterRpm;
                double currentShooterVelocity = robot.lShooter.getVelocity();
                if (Math.abs(currentShooterVelocity - targetShooterVelocity) <= SHOOTER_READY_WINDOW_TICKS
                        && (lastShooterInRangeRumbleTimeSec < 0.0
                        || (getRuntime() - lastShooterInRangeRumbleTimeSec) >= SHOOTER_IN_RANGE_RUMBLE_INTERVAL_SECONDS)) {
                    rumbleMechanismGamepad(SHOOTER_IN_RANGE_RUMBLE_MS);
                    lastShooterInRangeRumbleTimeSec = getRuntime();
                } else if (Math.abs(currentShooterVelocity - targetShooterVelocity) > SHOOTER_READY_WINDOW_TICKS) {
                    lastShooterInRangeRumbleTimeSec = -1.0;
                }

                if (!shooterLongRunRumbleSent
                        && shooterStartTimeSec >= 0.0
                        && (getRuntime() - shooterStartTimeSec) >= SHOOTER_LONG_RUN_SECONDS) {
                    rumbleMechanismGamepad(SHOOTER_LONG_RUN_RUMBLE_MS);
                    shooterLongRunRumbleSent = true;
                }
            } else {
                robot.setShooterPower(humanPlayer ? -0.3 : 0.0);
                shooterStartTimeSec = -1.0;
                shooterLongRunRumbleSent = false;
                lastShooterInRangeRumbleTimeSec = -1.0;
            }

            lastX = mechGP.x;
            lastIntakeToggleButton = mechGP.dpad_left;
            previousDpadDown = mechGP.dpad_down;
            previousDpadUp = mechGP.dpad_up;
            lastB = gamepad1.b;

            telemetry.addData("Distance Sensor", robot.distance1.getState());
            telemetry.addData("Controller Mode", isDualMode ? "Dual" : "Single");
            telemetry.addData("Limelight Targeting Toggle (G1 B)", bToggle);
            telemetry.addData("Auto Speed Toggle (X)", xToggle);
            telemetry.addData("Intake Toggle", intakeToggleOn ? "ON" : "OFF");
            telemetry.addData("Shooter Speed (Limelight)", getMotorSpeed(ta));
            telemetry.addData("Loop Time", elapsedTime.milliseconds());
            telemetry.update();
            elapsedTime.reset();
        }

        stopVisionThread();
        robot.prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
    }

    private Pipeline selectModeAndPipeline() {
        Pipeline selectedPipeline = Pipeline.MOTIF_PIPELINE;
        boolean modeSelected = false;
        boolean lastOptions = false;
        boolean lastLeftStickButton = false;

        telemetry.addLine("=== CONTROLLER MODE ===");
        telemetry.addLine("Press LEFT BUMPER: Single Driver (gamepad1 controls all)");
        telemetry.addLine("Press RIGHT BUMPER: Dual Driver (gamepad2 for mechanisms)");
        telemetry.addLine("");
        telemetry.addLine("After selecting mode, choose pipeline:");
        telemetry.addLine("X = Blue | B = Red | A = Motif");
        telemetry.update();

        while (opModeInInit()) {
            if (gamepad1.options && !lastOptions) {
                debugMode = !debugMode;
            }

            if (gamepad1.left_stick_button && !lastLeftStickButton) {
                humanPlayer = !humanPlayer;
            }

            if (!modeSelected) {
                if (gamepad1.left_bumper) {
                    isDualMode = false;
                    modeSelected = true;
                    sleep(Constants.TeleOp.DEBOUNCE_MS);
                } else if (gamepad1.right_bumper) {
                    isDualMode = true;
                    modeSelected = true;
                    sleep(Constants.TeleOp.DEBOUNCE_MS);
                }
            }

            if (modeSelected) {
                if (gamepad1.x) {
                    selectedPipeline = Pipeline.BLUE_PIPELINE;
                    robot.prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
                    break;
                }
                if (gamepad1.b) {
                    selectedPipeline = Pipeline.RED_PIPELINE;
                    robot.prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
                    break;
                }
                if (gamepad1.a) {
                    selectedPipeline = Pipeline.MOTIF_PIPELINE;
                    break;
                }
            }

            telemetry.addLine("=== CONTROLLER MODE ===");
            telemetry.addLine("Press LEFT BUMPER: Single Driver (gamepad1 controls all)");
            telemetry.addLine("Press RIGHT BUMPER: Dual Driver (gamepad2 for mechanisms)");
            telemetry.addLine("");
            telemetry.addLine("After selecting mode, choose pipeline:");
            telemetry.addLine("X = Blue | B = Red | A = Motif");
            telemetry.addData("Debug Mode", debugMode ? "Enabled" : "Disabled");
            telemetry.addData("Human Player", humanPlayer ? "Enabled" : "Disabled");
            telemetry.update();

            lastOptions = gamepad1.options;
            lastLeftStickButton = gamepad1.left_stick_button;
        }

        return selectedPipeline;
    }

    private void startVisionThread() {
        visionThread = new Thread(() -> {
            while (!Thread.currentThread().isInterrupted() && opModeIsActive() && !isStopRequested()) {
                latestResult = limelight.getLatestResult();
                try {
                    Thread.sleep(Constants.Vision.THREAD_SLEEP_MS);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }, "limelight-vision-thread");
        visionThread.start();
    }

    private void stopVisionThread() {
        if (visionThread != null && visionThread.isAlive()) {
            visionThread.interrupt();
        }
    }

    private void fetchVariablesFromLimelight(LLResult result) {
        if (result != null && result.isValid()) {
            tx = result.getTx();
            ty = result.getTy();
            ta = result.getTa();
        } else {
            tx = 0.0;
            ty = 0.0;
            ta = 0.0;
            telemetry.addData("Limelight", "No Targets");
        }
    }

    private double applyDeadband(double value, double threshold) {
        return Math.abs(value) < threshold ? 0.0 : value;
    }

    public double getMotorSpeed(double targetArea) {
        if (targetArea == 0.0) {
            return 0.0;
        }
        final double a = 0.2273;
        final double b = -0.8680;
        final double c = 0.49;
        return a * Math.exp(b * targetArea) + c;
    }

    private void rumbleMechanismGamepad(int durationMs) {
        Gamepad mechanismGamepad = getMechanismGamepad();
        if (mechanismGamepad != null) {
            mechanismGamepad.rumble(durationMs);
        }
    }
}
