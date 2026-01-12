package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Pipeline;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.List;


@TeleOp(name = "BaseBot Teleop")
public class BasebotTeleOp extends LinearOpMode {
    double ly = 0.0;
    double lx = 0.0;
    double rx = 0.0;
    private static final double TARGET_FEET = 3.875;
    private static final double RAD_LAUNCH_ANGLE = Math.toRadians(37.84);
    private static final double GRAVITY = 32.2; // in ft^2 per second
    private double targetDistance = 0.0;

    double velocity;


    double tx;
    double ty;
    double ta;

    RobotHardware robot;
    Limelight3A limelight;

    @Override
    public void runOpMode() {
        // Shooter State
        double shooterPower = 0.7;


        boolean previousDpadUp = false;
        boolean previousDpadDown = false;
        boolean previousX = false;
        boolean previousY = false;
        boolean shooterOn = false;
        boolean previous_left = false;
        boolean previous_right = false;
        boolean left_toggle = false;
        boolean right_toggle = false;
        double rxModifier = 0.0;

        // Gear Calc
        double gearRatio = (double) 30/24;
        double targetRPM = 200;
        final double COUNTS_PER_MOTOR_REV = 28;
        final double TICKS_PER_REV = COUNTS_PER_MOTOR_REV * gearRatio;


        Pipeline selectedPipeline = Pipeline.MOTIF_PIPELINE; // Now an enum

        boolean bToggle = false;
        boolean lastB = false;

        boolean xToggle = false;
        boolean lastX = false;


        robot = new RobotHardware(hardwareMap, new Pose2d(0, 0, 180));
        limelight = robot.limelight;

        robot.prism.setStripLength(29);

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
            double rawVelocity = robot.lShooter.getVelocity();
            double rpm = rawVelocity / TICKS_PER_REV * 60;
            telemetry.addData("Actual RPM", rpm);
            telemetry.addData("Shooter RPM", targetRPM);

//            targetRPM = ((120 * velocity) / (Math.PI * 0.315)) * 1.2;
            double velocityTarget = (targetRPM * TICKS_PER_REV) / 60;


//            if (rpm < targetRPM - 40) {
//                shooterPower += 0.0001;
//                telemetry.addLine("Speed Up");
//            } else if (rpm > targetRPM + 40) {
//                shooterPower -= 0.0001;
//                telemetry.addLine("Speed Down");
//            }
            velocity = Math.sqrt(
                    (GRAVITY * Math.pow(targetDistance, 2)/
                            (2*Math.cos(RAD_LAUNCH_ANGLE)*Math.cos(RAD_LAUNCH_ANGLE)*
                                    (targetDistance*Math.tan(RAD_LAUNCH_ANGLE)-TARGET_FEET))));

            LLResult result = limelight.getLatestResult();

            fetchVariablesFromLimelight(result);

            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            int i;
            for (i = 0; i < fiducialResults.size(); i++) {
                if (!fiducialResults.isEmpty()) {
                    telemetry.addData("Tag ID " + i, fiducialResults.get(i).getFiducialId());
                }
            }

            if (bToggle) {
                rxModifier = ((tx/27.25) * 0.8);
            } else {
                rxModifier = 0.0;
            }

            if (!(tx == 0.0 && ty == 0.0 && ta == 0.0)) {
                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
            }

            ly = -gamepad1.left_stick_y;
            lx = gamepad1.left_stick_x;

            if (bToggle) {
                rx = gamepad1.right_stick_x * 0.85 + rxModifier;
            } else {
                rx = gamepad1.right_stick_x * 0.85;
            }

            double denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1.0);
            double frontLeftPower = (ly + lx + rx) / denominator;
            double backLeftPower = (ly - lx + rx) / denominator;
            double frontRightPower = (ly - lx - rx) / denominator;
            double backRightPower = (ly + lx - rx) / denominator;

            double slowdown = 0.85;
            robot.frontLeft.setPower(frontLeftPower * slowdown);
            robot.frontRight.setPower(frontRightPower * slowdown);
            robot.backLeft.setPower(backLeftPower * slowdown);
            robot.backRight.setPower(backRightPower * slowdown);
//            telemetry.addData("distance in mm", getDistanceMM(robot.distance1.getVoltage()));
            telemetry.addData("detected", robot.distance1.getState());


            // --- Consolidated Intake and Index Control (RT for Shooting is highest priority) ---
            double indexPower = 0.0;
            double intakePower = 0.0;
            final double RT_THRESHOLD = 0.1;

            if (gamepad1.dpad_left && !lastGamepad.dpad_left) {
                left_toggle = !left_toggle;
            }


            if (gamepad1.right_trigger > RT_THRESHOLD) {
                // RT pressed: Shoot (Index feed, Intake stop)
                // Shooter must be primed (shooterOn = true) for the index to run.
                if (shooterOn) {
                    indexPower = 1.0;
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

            // Update previous state for momentary button presses
            // --------------------------------------------------------------------------------

            if (gamepad1.b && !lastGamepad.b) {
                bToggle = !bToggle;
            }

            // D-pad shooter control logic
            if (gamepad1.dpad_up && !lastGamepad.dpad_up) {
//                shooterPower = Math.min(1.0, shooterPower + 0.01); // Clamp power at 1.0
                targetRPM += 50;
//                targetDistance += 0.25;
            }

            if (gamepad1.dpad_down && !lastGamepad.dpad_down) {
//                shooterPower = Math.max(0.0, shooterPower - 0.01); // Clamp power at 0.0
                targetRPM -= 50;
//                targetDistance += 0.25;
            }

            telemetry.addData("Target RPM", targetRPM);

            if (gamepad1.right_bumper) {
                shooterOn = true;
            } else if (gamepad1.left_bumper) {
                shooterOn = false;
            }

            // Xtoggle
            if (gamepad1.x && !lastGamepad.x) {
                xToggle = !xToggle;
            }

            if (shooterOn) {
                if (!xToggle) {
                    robot.lShooter.setVelocity(velocityTarget);
                    robot.rShooter.setVelocity(velocityTarget);
                } else {
                    robot.setShooterPower(getMotorSpeed(ta));
                }
            } else {
                // If not shooterOn, ensure motors are off
                robot.setShooterPower(0.0);
            }

            lastGamepad = gamepad1;

            // Update Telemetry
            telemetry.addData("Targeting Toggle", bToggle);
            telemetry.addData("Auto Speed", xToggle);
            telemetry.addData("speed for 'auto' ", getMotorSpeed(ta));
            telemetry.addData("Shooter Power", shooterPower);
            telemetry.update();
        }
    }
    void fetchVariablesFromLimelight(LLResult result) {
        if (result != null && result.isValid()) {
            tx = result.getTx(); // How far left or right the target is (degrees)
            ty = result.getTy(); // How far up or down the target is (degrees)
            ta = result.getTa(); // How much of the image the target is (0%-100% of the image)
        } else {
            telemetry.addData("Limelight", "No Targets");
            tx = 0.0;
            ty = 0.0;
            ta = 0.0;
        }
    }

    public double getMotorSpeed(double targetArea) {
        if (targetArea == 0) {
            return 0.0;
        }
//        final double A = 0.248;
//        final double B = -0.5709;
//        final double C = 0.4450;
        final double A = 0.2273;
        final double B = -0.8680;
        final double C = 0.49;
        return A * Math.exp(B * targetArea) + C;
    }
    public double getDistanceFromTag(double targetArea) {
        return targetArea * 66;
    }

//    double getDistanceMM(double voltage) {
//        final double MAX_VOLTS = 3.3;
//        final double MAX_DISTANCE_MM = 1000.0;
//        return (voltage / MAX_VOLTS) * MAX_DISTANCE_MM;
//    }
}