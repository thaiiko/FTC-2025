package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotState;

/**
 * OpMode to help calibrate the shooter angle.
 * 
 * Instructions:
 * 1. Position the robot at a known distance from the target.
 * 2. Use D-pad Left/Right to set that distance in the OpMode.
 * 3. Use D-pad Up/Down to adjust the shooter RPM.
 * 4. Shoot balls using the 'A' button.
 * 5. Adjust the RPM until the balls consistently hit the target.
 * 6. The OpMode will display the implied shooter angle.
 * 7. Update the LAUNCH_ANGLE_DEG constant in BasebotUnifiedTeleOp with this value.
 */
@TeleOp(name = "Shooter Angle Calibration", group = "Calibration")
public class ShooterAngleCalibration extends LinearOpMode {
    // Constants from BasebotUnifiedTeleOp
    private static final double TARGET_FEET_DEFAULT = 3.875; 
    private static final double GRAVITY_FT_S2 = 32.2;
    private static final double SHOOTER_WHEEL_DIAMETER_FT = 0.315;
    private static final double RPM_EMPIRICAL_FACTOR = 1.2;
    private static final double RPM_MAGIC_CONSTANT = 60.0;

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(hardwareMap, RobotState.getCurrentPose());
        
        // Ensure motors are ready for velocity control
        robot.lShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double gearRatio = (double) 30/24;
        final double COUNTS_PER_MOTOR_REV = 28;
        final double TICKS_PER_REV = COUNTS_PER_MOTOR_REV * gearRatio;

        double testRPM = 1500;
        double testDistanceFeet = 8.0;
        double targetHeightFeet = TARGET_FEET_DEFAULT;
        
        boolean prevUp = false;
        boolean prevDown = false;
        boolean prevLeft = false;
        boolean prevRight = false;
        boolean prevLB = false;
        boolean prevRB = false;

        boolean shooterOn = false;

        telemetry.addLine("Ready. Press Start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Adjust RPM (UP/DOWN)
            if (gamepad1.dpad_up && !prevUp) testRPM += 50;
            if (gamepad1.dpad_down && !prevDown) testRPM -= 50;
            prevUp = gamepad1.dpad_up;
            prevDown = gamepad1.dpad_down;

            // Adjust Distance (LEFT/RIGHT)
            if (gamepad1.dpad_right && !prevRight) testDistanceFeet += 0.25;
            if (gamepad1.dpad_left && !prevLeft) testDistanceFeet -= 0.25;
            prevRight = gamepad1.dpad_right;
            prevLeft = gamepad1.dpad_left;
            
            // Adjust Height (LB/RB)
            if (gamepad1.left_bumper && !prevLB) targetHeightFeet -= 0.1;
            if (gamepad1.right_bumper && !prevRB) targetHeightFeet += 0.1;
            prevLB = gamepad1.left_bumper;
            prevRB = gamepad1.right_bumper;

            // Calculation of launch velocity from RPM
            // velocity = (motorRpm * PI * D) / (RPM_MAGIC_CONSTANT * RPM_EMPIRICAL_FACTOR)
            double v = (testRPM * Math.PI * SHOOTER_WHEEL_DIAMETER_FT) / (RPM_MAGIC_CONSTANT * RPM_EMPIRICAL_FACTOR);
            
            double x = testDistanceFeet;
            double y = targetHeightFeet;
            double g = GRAVITY_FT_S2;

            // Solve for tan(theta) using the quadratic:
            // (g * x^2) * tan^2(theta) - (2 * v^2 * x) * tan(theta) + (g * x^2 + 2 * v^2 * y) = 0
            double a = g * Math.pow(x, 2);
            double b = -2 * Math.pow(v, 2) * x;
            double c = g * Math.pow(x, 2) + 2 * Math.pow(v, 2) * y;

            double discriminant = Math.pow(b, 2) - 4 * a * c;
            
            // Toggle Shooter
            if (gamepad1.b) shooterOn = true;
            if (gamepad1.x) shooterOn = false;

            if (shooterOn) {
                double motorVelocityTicks = testRPM * TICKS_PER_REV / 60.0;
                robot.lShooter.setVelocity(motorVelocityTicks);
                robot.rShooter.setVelocity(motorVelocityTicks);
            } else {
                robot.setShooterPower(0);
            }

            // Fire (Index)
            if (gamepad1.a) {
                robot.index.setPower(1.0);
            } else {
                robot.index.setPower(0);
            }

            telemetry.addLine("--- Shooter Angle Calibration ---");
            telemetry.addLine("D-pad U/D: RPM | D-pad L/R: Dist | LB/RB: Height");
            telemetry.addLine("B: Shooter ON | X: Shooter OFF | A: Fire");
            telemetry.addLine("");
            telemetry.addData("Test RPM", "%.0f", testRPM);
            telemetry.addData("Distance (ft)", "%.2f", testDistanceFeet);
            telemetry.addData("Height Diff (ft)", "%.2f", targetHeightFeet);
            telemetry.addData("Launch Velocity (ft/s)", "%.2f", v);
            
            if (discriminant >= 0) {
                double t1 = (-b + Math.sqrt(discriminant)) / (2 * a);
                double t2 = (-b - Math.sqrt(discriminant)) / (2 * a);
                double angle1 = Math.toDegrees(Math.atan(t1));
                double angle2 = Math.toDegrees(Math.atan(t2));
                telemetry.addData("Implied Angle (High arc)", "%.2f deg", angle1);
                telemetry.addData("Implied Angle (Low arc)", "%.2f deg", angle2);
                telemetry.addLine("\nUse the angle that matches your physical setup.");
            } else {
                telemetry.addLine("\nERROR: RPM too low to reach target!");
            }
            
            telemetry.addData("Shooter Status", shooterOn ? "ON" : "OFF");
            telemetry.addData("Current Velocity (Ticks/s)", robot.lShooter.getVelocity());
            telemetry.update();
        }
    }
}
