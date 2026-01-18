package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Pipeline;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotState;

import java.util.List;


@TeleOp(name = "BaseBot Teleop")
public class BasebotTeleOp extends LinearOpMode {
    // --- Constants for Ballistic Solver ---
    private static final double TARGET_FEET = 3.875; // Target height above launcher in feet
    private static final double LAUNCH_ANGLE_DEG = 48;
    private static final double LAUNCH_ANGLE_RAD = Math.toRadians(LAUNCH_ANGLE_DEG);
    private static final double GRAVITY_FT_S2 = 32.2; // Gravity in ft/s^2
    private static final double SHOOTER_WHEEL_DIAMETER_FT = 0.315; // Diameter of shooter wheel in feet
    private static final double RPM_EMPIRICAL_FACTOR = 1.2;
    private static final double RPM_MAGIC_CONSTANT = 120.0; // Likely a gearing or empirical factor, replacing raw 120.0

    // --- State Variables ---
    private double targetDistance; // Calculated distance to target in feet
    double velocity; // Calculated launch velocity in ft/s

    double tx;
    double ty;
    double ta;

    RobotHardware robot;
    Limelight3A limelight;

    @Override
    public void runOpMode() {
        // --- Initialization and Toggles ---
        double shooterPower = 0.7;

        boolean previousDpadUp = false;
        boolean previousDpadDown = false;
        boolean previousY = false; // For gamepad1.y (Index Reverse)
        boolean shooterOn = false;

        // --- Shooter Motor Ticks Conversion ---
        double gearRatio = (double) 30/24;
        double targetRPM = 200; // Used as a base for velocity control display
        final double COUNTS_PER_MOTOR_REV = 28;
        final double TICKS_PER_REV = COUNTS_PER_MOTOR_REV * gearRatio;

        Pipeline selectedPipeline = Pipeline.MOTIF_PIPELINE;

        boolean bToggle = false; // Limelight Steering Toggle (gamepad1.b)
        boolean lastB = false;

        boolean xToggle = false; // Auto Speed Toggle (gamepad1.x)
        boolean lastX = false;


        robot = new RobotHardware(hardwareMap, RobotState.getCurrentPose());
        limelight = robot.limelight;

        robot.prism.setStripLength(29);

        // --- Pipeline Selection in Init ---
        while (opModeInInit()) {
            if (gamepad1.x) {
                selectedPipeline = Pipeline.BLUE_PIPELINE;
                robot.prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
                break;
            } else if (gamepad1.b) {
                selectedPipeline = Pipeline.RED_PIPELINE;
                robot.prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
                break;
            } else if (gamepad1.a) {
                selectedPipeline = Pipeline.MOTIF_PIPELINE;
                break;
            }
        }

        telemetry.addData("Selected Pipeline", selectedPipeline.name());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Gamepad lastGamepad = new Gamepad();

        limelight.pipelineSwitch(selectedPipeline.getValue());
        waitForStart();

        while (opModeIsActive()) {
            // --- Shooter Telemetry and Motor Velocity Calculations ---
            double rawVelocity = robot.lShooter.getVelocity();
            double currentRpm = rawVelocity / TICKS_PER_REV * 60;
            telemetry.addData("Target RPM (Display)", targetRPM);

            // Ballistic solver: calculates required velocity (v) in ft/s
            Pose2d blueTower = new Pose2d(-60, -57, 0); // Tower position in inches (assumed)

            double distanceToTowerInches = Math.sqrt(
                Math.pow(robot.localizer.getPose().position.x - blueTower.position.x, 2) +
                Math.pow(robot.localizer.getPose().position.y - blueTower.position.y, 2)
            );
            // Convert calculated distance from inches (RoadRunner default) to feet
            targetDistance = distanceToTowerInches / 12.0; 

            // The main ballistic equation (solved for v^2)
            // v^2 = (g * x^2) / (2 * cos^2(theta) * (x * tan(theta) - y))
            double numerator = GRAVITY_FT_S2 * Math.pow(targetDistance, 2);
            double denominator = 2 * Math.cos(LAUNCH_ANGLE_RAD) * Math.cos(LAUNCH_ANGLE_RAD)
                    * (targetDistance * Math.tan(LAUNCH_ANGLE_RAD) - TARGET_FEET);

            if (targetDistance <= 0 || denominator <= 0) {
                // Denominator <= 0 means the target is too high/far for the fixed angle.
                telemetry.addLine("Invalid shot geometry (too high/far for angle): velocity set to 0.0");
                velocity = 0.0;
            } else {
                velocity = Math.sqrt(numerator / denominator);
            }

            // Convert ballistic velocity (ft/s) to motor velocity (ticks/s)
            double motorRpm = (RPM_MAGIC_CONSTANT * velocity) / (Math.PI * SHOOTER_WHEEL_DIAMETER_FT) * RPM_EMPIRICAL_FACTOR;
            double motorVelocity = motorRpm * TICKS_PER_REV / 60;

            telemetry.addData("Current RPM", currentRpm);
            telemetry.addData("Required Launch Velocity (ft/s)", velocity);
            telemetry.addData("Target Distance (feet)", targetDistance);
            telemetry.addData("Required Motor Velocity (ticks/s)", motorVelocity);
            telemetry.addData("Launch Angle", "deg=" + LAUNCH_ANGLE_DEG);

            // --- Limelight Data Fetch ---
            LLResult result = limelight.getLatestResult();
            fetchVariablesFromLimelight(result);

            // Fiducial (Tag) Results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (int i = 0; i < fiducialResults.size(); i++) {
                if (!fiducialResults.isEmpty()) {
                    telemetry.addData("Tag ID " + i, fiducialResults.get(i).getFiducialId());
                }
            }

            // --- Drive Control (RoadRunner PoseVelocity) ---
            
            // Limelight Steering Assist Calculation for Rotation
            double rxModifier = 0.0;
            if (bToggle) {
                // 0.4 coefficient taken from user's sample code
                rxModifier = ((tx/27.25) * 0.8);
            }

            double turn = -gamepad1.right_stick_x * 0.6; // Base rotation speed
            turn += rxModifier; // Add Limelight assist

            // Telemetry for Limelight Data
            if (!(tx == 0.0 && ty == 0.0 && ta == 0.0)) {
                telemetry.addData("Target X (tx)", tx);
                telemetry.addData("Target Y (ty)", ty);
                telemetry.addData("Target Area (ta)", ta);
            }

            // Apply powers with cubic scaling for translation (x/y)
            robot.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -1.33 * Math.pow(gamepad1.left_stick_y, 3), // Axial (y)
                            -1.33 * Math.pow(gamepad1.left_stick_x, 3)  // Lateral (x)
                    ),
                    turn // Rotational (heading)
            ));
            
            /*
            // --- OLD DRIVE CONTROL LOGIC (Mecanum Math) ---
            
            // NOTE: rxModifier was previously declared as a local variable here (double rxModifier = 0.0) 
            // and is now declared above in the new drive control section.

            // Limelight Steering Assist Calculation (Using old 0.8 coefficient)
            // if (bToggle) {
            //     rxModifier = ((tx/27.25) * 0.8);
            // } else {
            //     rxModifier = 0.0;
            // }

            // Telemetry for Limelight Data
            // NOTE: Telemetry is now placed in the new active Drive Control section.
            // if (!(tx == 0.0 && ty == 0.0 && ta == 0.0)) {
            //     telemetry.addData("Target X (tx)", tx);
            //     telemetry.addData("Target Y (ty)", ty);
            //     telemetry.addData("Target Area (ta)", ta);
            // }

            // Get stick inputs
            // ly = -gamepad1.left_stick_y; // Variable removed
            // lx = gamepad1.left_stick_x; // Variable removed

            // Calculate rotation (rx), applying Limelight assist if toggled
            // if (bToggle) {
            //     rx = -gamepad1.right_stick_x * 0.85 + (rxModifier * 0.3);
            // } else {
            //     rx = -gamepad1.right_stick_x * 0.85;
            // }

            // Mecanum Drive Math
            // double divisor = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1.0);
            // double frontLeftPower = (ly + lx + rx) / divisor;
            // double backLeftPower = (ly - lx + rx) / divisor;
            // double frontRightPower = (ly - lx - rx) / divisor;
            // double backRightPower = (ly + lx - rx) / divisor;

            // Apply power with slowdown
            // final double SLOWDOWN_FACTOR = 0.85;
            // robot.frontLeft.setPower(frontLeftPower * SLOWDOWN_FACTOR);
            // robot.frontRight.setPower(frontRightPower * SLOWDOWN_FACTOR);
            // robot.backLeft.setPower(backLeftPower * SLOWDOWN_FACTOR);
            // robot.backRight.setPower(backRightPower * SLOWDOWN_FACTOR);
            */

            robot.updatePoseEstimate();
            telemetry.addData("Distance Sensor", robot.distance1.getState());


            // --- Consolidated Intake and Index Control ---
            double indexPower = 0.0;
            double intakePower = 0.0;
            final double RT_THRESHOLD = 0.1;

            // Control based on Gamepad 1 (Mechanism controls ported from Gamepad 2)
            if (gamepad1.right_trigger > RT_THRESHOLD) {
                // RT pressed: Shoot (Index feed, Intake stop)
                if (shooterOn) {
                    indexPower = 1.0; // Index feed
                    intakePower = 0.0; // Stop intake while shooting
                }
            } else if (gamepad1.dpad_right) {
                // D-pad Right: Intake Reverse/Outtake
                intakePower = -0.8;
                indexPower = 0.0;
            } else if (gamepad1.dpad_left) {
                // D-pad Left: Intake
                intakePower = 0.8;
                indexPower = 0.0;
            } else if (gamepad1.y && !lastGamepad.y) {
                // Gamepad Y: Index Reverse (to clear jams - momentary press)
                indexPower = -0.5;
                intakePower = 0.0;
            }

            robot.index.setPower(indexPower);
            robot.intake.setPower(intakePower);

            // --- Shooter Power Adjustments (Gamepad 1 DPAD) ---
            if (gamepad1.dpad_up && !previousDpadUp) {
                shooterPower = Math.min(1.0, shooterPower + 0.01);
            }

            if (gamepad1.dpad_down && !previousDpadDown) {
                shooterPower = Math.max(0.0, shooterPower - 0.01);
            }

            telemetry.addData("Manual Shooter Power", shooterPower);

            // Shooter On/Off Toggles
            if (gamepad1.right_bumper) {
                shooterOn = true;
            } else if (gamepad1.left_bumper) {
                shooterOn = false;
            }

            // Auto/Manual Speed Toggle (gamepad1.x)
            if (gamepad1.x && !lastX) {
                xToggle = !xToggle;
            }

            if (gamepad1.b && !lastB) {
                bToggle = !bToggle;
            }

            // Apply Shooter Power
            if (shooterOn) {
                if (!xToggle) {
                    // Manual Power Mode
                    robot.lShooter.setPower(shooterPower);
                    robot.rShooter.setPower(shooterPower);
                } else {
                    // Auto Velocity Mode (Ballistic Solver)
                    robot.lShooter.setVelocity(motorVelocity);
                    robot.rShooter.setVelocity(motorVelocity);
                }
            } else {
                // Ensure motors are off when not shooting
                robot.setShooterPower(0.0);
            }

            // --- Update Toggles/Previous States ---
            lastX = gamepad1.x;
            previousDpadDown = gamepad1.dpad_down;
            previousDpadUp = gamepad1.dpad_up;
            lastB = gamepad1.b;
            previousY = gamepad1.y;

            // --- Final Telemetry Update ---
            telemetry.addData("Limelight Targeting Toggle (G1 B)", bToggle);
            telemetry.addData("Auto Speed Toggle (G1 X)", xToggle);
            telemetry.addData("Shooter Speed (Limelight)", getMotorSpeed(ta));
            telemetry.update();
        }
    }
    
    // --- Helper Methods ---

    /**
     * Fetches Limelight variables (tx, ty, ta) from the result.
     * @param result The latest LLResult from the Limelight.
     */
    void fetchVariablesFromLimelight(LLResult result) {
        if (result != null && result.isValid()) {
            tx = result.getTx(); // Horizontal Offset (degrees)
            ty = result.getTy(); // Vertical Offset (degrees)
            ta = result.getTa(); // Target Area (0%-100% of the image)
        } else {
            telemetry.addData("Limelight", "No Targets");
            tx = 0.0;
            ty = 0.0;
            ta = 0.0;
        }
    }

    /**
     * Returns an empirical motor speed based on target area (ta).
     * This is likely a fallback or a simpler targeting method, not the ballistic solver.
     * @param targetArea The target area (ta) from the Limelight.
     * @return The calculated motor speed (likely power or normalized velocity).
     */
    public double getMotorSpeed(double targetArea) {
        if (targetArea == 0) {
            return 0.0;
        }
        // Empirical curve fit: A * exp(B * ta) + C
        final double A = 0.2273;
        final double B = -0.8680;
        final double C = 0.49;
        return A * Math.exp(B * targetArea) + C;
    }
    
    // Kept the original helper, but it seems unused/deprecated by the main solver logic.
    public double getDistanceFromTag(double targetArea) {
        return targetArea * 66; 
    }
}