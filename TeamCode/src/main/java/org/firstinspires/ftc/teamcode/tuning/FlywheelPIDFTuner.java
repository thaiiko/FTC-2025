package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Flywheel PIDF Tuner for lShooter and rShooter motors.
 *
 * Controls:
 * - Gamepad1 Dpad Up/Down: Adjust kP (Proportional)
 * - Gamepad1 Dpad Left/Right: Adjust kF (FeedForward)
 * - Gamepad1 Left Bumper/Right Bumper: Adjust target velocity
 * - Gamepad1 A: Toggle flywheel on/off
 * - Gamepad1 Y: Reset PIDF to default values
 * - Gamepad1 X: Fine adjustment mode toggle (smaller increments)
 */
@TeleOp(name = "Flywheel PIDF Tuner", group = "Tuning")
public class FlywheelPIDFTuner extends LinearOpMode {

    // Target velocity in ticks per second
    private double targetVelocity = 1500.0;

    // Adjustment increments
    private static final double P_INCREMENT_COARSE = 1.0;
    private static final double P_INCREMENT_FINE = 0.1;
    private static final double F_INCREMENT_COARSE = 1.0;
    private static final double F_INCREMENT_FINE = 0.1;
    private static final double VELOCITY_INCREMENT = 100.0;

    // Motor encoder constants
    private static final double GEAR_RATIO = 30.0 / 24.0;
    private static final double COUNTS_PER_MOTOR_REV = 28.0;
    private static final double TICKS_PER_REV = COUNTS_PER_MOTOR_REV * GEAR_RATIO;

    // State variables
    private boolean flywheelOn = false;
    private boolean fineMode = false;

    // Previous button states for edge detection
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    private boolean prevLeftBumper = false;
    private boolean prevRightBumper = false;
    private boolean prevA = false;
    private boolean prevY = false;
    private boolean prevX = false;

    @Override
    public void runOpMode() {
        // Initialize motors
        DcMotorEx lShooter = hardwareMap.get(DcMotorEx.class, "lShooter");
        DcMotorEx rShooter = hardwareMap.get(DcMotorEx.class, "rShooter");

        // Set motor directions (matching RobotHardware configuration)
        lShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rShooter.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set run mode to use encoder for velocity control
        lShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set zero power behavior
        lShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Get default PIDF coefficients from the motor
        PIDFCoefficients defaultCoeffs = lShooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        // PIDF coefficients (P and F only, I and D set to 0)
        double kP = defaultCoeffs.p;
        // Not used, but required for PIDFCoefficients
        double kI = 0.0;  // We only care about P and F
        // Not used, but required for PIDFCoefficients
        double kD = 0.0;
        double kF = defaultCoeffs.f;

        telemetry.addLine("=== Flywheel PIDF Tuner ===");
        telemetry.addLine("Press START to begin");
        telemetry.addLine("");
        telemetry.addLine("Controls:");
        telemetry.addLine("Dpad Up/Down: Adjust kP");
        telemetry.addLine("Dpad Left/Right: Adjust kF");
        telemetry.addLine("LB/RB: Adjust target velocity");
        telemetry.addLine("A: Toggle flywheel");
        telemetry.addLine("Y: Reset to defaults");
        telemetry.addLine("X: Toggle fine mode");
        telemetry.addData("Default kP", defaultCoeffs.p);
        telemetry.addData("Default kF", defaultCoeffs.f);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Handle button inputs with edge detection ---

            // Toggle fine mode (X button)
            if (gamepad1.x && !prevX) {
                fineMode = !fineMode;
            }

            double pIncrement = fineMode ? P_INCREMENT_FINE : P_INCREMENT_COARSE;
            double fIncrement = fineMode ? F_INCREMENT_FINE : F_INCREMENT_COARSE;

            // Adjust kP (Dpad Up/Down)
            if (gamepad1.dpad_up && !prevDpadUp) {
                kP += pIncrement;
            }
            if (gamepad1.dpad_down && !prevDpadDown) {
                kP = Math.max(0, kP - pIncrement);
            }

            // Adjust kF (Dpad Left/Right)
            if (gamepad1.dpad_right && !prevDpadRight) {
                kF += fIncrement;
            }

            if (gamepad1.dpad_left && !prevDpadLeft) {
                kF = Math.max(0, kF - fIncrement);
            }

            // Adjust target velocity (Bumpers)
            if (gamepad1.right_bumper && !prevRightBumper) {
                targetVelocity += VELOCITY_INCREMENT;
            }
            if (gamepad1.left_bumper && !prevLeftBumper) {
                targetVelocity = Math.max(0, targetVelocity - VELOCITY_INCREMENT);
            }

            // Toggle flywheel on/off (A button)
            if (gamepad1.a && !prevA) {
                flywheelOn = !flywheelOn;
            }

            // Reset to defaults (Y button)
            if (gamepad1.y && !prevY) {
                PIDFCoefficients defaults = lShooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                kP = defaults.p;
                kF = defaults.f;
                targetVelocity = 1500.0;
            }

            // Update previous button states
            prevDpadUp = gamepad1.dpad_up;
            prevDpadDown = gamepad1.dpad_down;
            prevDpadLeft = gamepad1.dpad_left;
            prevDpadRight = gamepad1.dpad_right;
            prevLeftBumper = gamepad1.left_bumper;
            prevRightBumper = gamepad1.right_bumper;
            prevA = gamepad1.a;
            prevY = gamepad1.y;
            prevX = gamepad1.x;

            // --- Apply PIDF coefficients to both motors ---
            PIDFCoefficients newCoeffs = new PIDFCoefficients(kP, kI, kD, kF);
            lShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newCoeffs);
            rShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newCoeffs);

            // --- Control flywheel ---
            if (flywheelOn) {
                lShooter.setVelocity(targetVelocity);
                rShooter.setVelocity(targetVelocity);
            } else {
                lShooter.setVelocity(0);
                rShooter.setVelocity(0);
            }

            // --- Get current velocities ---
            double lVelocity = lShooter.getVelocity();
            double rVelocity = rShooter.getVelocity();
            double avgVelocity = (lVelocity + rVelocity) / 2.0;

            // Calculate RPM for display
            double lRPM = (lVelocity / TICKS_PER_REV) * 60.0;
            double rRPM = (rVelocity / TICKS_PER_REV) * 60.0;
            double avgRPM = (lRPM + rRPM) / 2.0;
            double targetRPM = (targetVelocity / TICKS_PER_REV) * 60.0;

            // Calculate error
            double error = targetVelocity - avgVelocity;
            double errorPercent = targetVelocity > 0 ? (error / targetVelocity) * 100.0 : 0;

            // --- Telemetry ---
            telemetry.addLine("=== Flywheel PIDF Tuner ===");
            telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
            telemetry.addData("Fine Mode", fineMode ? "ON (0.1)" : "OFF (1.0)");
            telemetry.addLine("");

            telemetry.addLine("--- PIDF Coefficients ---");
            telemetry.addData("kP (Dpad U/D)", "%.2f", kP);
            telemetry.addData("kF (Dpad L/R)", "%.2f", kF);
            telemetry.addLine("");

            telemetry.addLine("--- Target ---");
            telemetry.addData("Target Velocity (ticks/s)", "%.1f", targetVelocity);
            telemetry.addData("Target RPM", "%.1f", targetRPM);
            telemetry.addLine("");

            telemetry.addLine("--- Current Velocities ---");
            telemetry.addData("Left Velocity (ticks/s)", "%.1f", lVelocity);
            telemetry.addData("Right Velocity (ticks/s)", "%.1f", rVelocity);
            telemetry.addData("Average Velocity (ticks/s)", "%.1f", avgVelocity);
            telemetry.addLine("");

            telemetry.addLine("--- RPM ---");
            telemetry.addData("Left RPM", "%.1f", lRPM);
            telemetry.addData("Right RPM", "%.1f", rRPM);
            telemetry.addData("Average RPM", "%.1f", avgRPM);
            telemetry.addLine("");

            telemetry.addLine("--- Error ---");
            telemetry.addData("Velocity Error (ticks/s)", "%.1f", error);
            telemetry.addData("Error %", "%.2f%%", errorPercent);
            telemetry.addLine("");

            telemetry.addLine("--- Controls ---");
            telemetry.addLine("A: Toggle flywheel | Y: Reset");
            telemetry.addLine("X: Toggle fine mode");

            telemetry.update();
        }

        // Stop motors when OpMode ends
        lShooter.setVelocity(0);
        rShooter.setVelocity(0);
    }
}

